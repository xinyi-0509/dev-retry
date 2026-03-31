# LibHGP 可视化模块 Bug 修复技术文档

**涉及模块**：`hull3D.cpp`、`vis/backend/app.py`、`vis/backend/web/index.html`  
**修复日期**：2026-03-31  
**修复人**：xinyi-0509

---

## 概述

本文档记录了 LibHGP 可视化模块在开发过程中发现并修复的全部 9 个 Bug，按发现时间顺序排列。这些 Bug 涉及 C++ 底层几何算法、Python 后端接口、JavaScript 前端渲染三个层次，部分 Bug 之间存在因果传播关系。

### Bug 一览表

| # | 模块 | 问题描述 | 严重程度 |
|---|---|---|---|
| 1 | C++ `hull3D.cpp` | 凸包两次调用 `NewtonApple_hull_3D` 导致缺失面片 | 🔴 严重 |
| 2 | 前端 `index.html` | 测地线选点强制吸附到最近顶点 | 🟡 中 |
| 3 | 后端 `app.py` | 切片轮廓强制拼接导致长连线 | 🟡 中 |
| 4 | 后端 `app.py` | `save_mesh` 未同时写 OFF 文件 | 🔴 严重 |
| 5 | 后端 `app.py` | 上传 OBJ 未修复非流形网格导致 CGAL 崩溃 | 🔴 严重 |
| 6 | 前端 `index.html` | 测地线失败时前端崩溃，错误信息不可读 | 🟡 中 |
| 7 | 后端+前端 | 切片接口用 `offsets` 导致轮廓展平、归属丢失 | 🔴 严重 |
| 8 | 前端 `index.html` | 切片 `plane_ds` 符号错误，切割平面全在模型外 | 🔴 严重 |
| 9 | 后端 `app.py` | 多部件模型测地线返回 -1，无友好提示 | 🟢 低 |

---

## Bug 1：凸包底层两次调用 `NewtonApple_hull_3D` 导致面片缺失

### 问题现象

使用 `/api/convex_hull_3d` 接口生成随机点的 3D 凸包时，部分情况下网格**缺少若干面片**，在可视化工具中出现明显破洞。该问题具有随机性，特定分布的点集会稳定复现缺失 2～4 个面片的情况。

### 根本原因

`HGP_3D_Convex_Hulls_C2` 的 C++ 实现中，`NewtonApple_hull_3D` 被调用了两次：

```cpp
// 第一次调用（结果正确）
int nx = de_duplicateR3(pts, outx, pts2);
int ts = NewtonApple_hull_3D(pts2, tris);

// 把第一次结果展开成点列表
for (int i = 0; i < (int)tris.size(); i++) {
    pts.push_back(pts2[tris[i].a]);
    pts.push_back(pts2[tris[i].b]);
    pts.push_back(pts2[tris[i].c]);
}

// 清空后对展开点再去重
pts2.clear(); outx.clear(); tris.clear();
nx = de_duplicateR3(pts, outx, pts2);

// 第二次调用（结果不完整）← Bug 根源
ts = NewtonApple_hull_3D(pts2, tris);
```

原始设计意图是通过"展开→去重→二次调用"修正法向不一致问题，但带来了精度损失：`R3` 结构体用 `float` 存储坐标，而输入是 `double`，经历 `double→float→double` 的精度往返后，第二次调用面对的点集与原始点集存在微小差异，在某些几何配置下输出不完整的凸包。

### 错误传播链

```
Bug 1: 凸包缺失面片 → 网格不封闭
        ↓
CGAL 测地线在不封闭网格上崩溃
        ↓ 误判为"需要吸附到顶点"
Bug 2: 前端加 snap 补丁（治标不治本）
        ↓
CGAL 切片在破洞区域产生异常段
        ↓ 误判为"需要拼接轮廓"
Bug 3: 后端加 join_contour_segments（治标不治本）
        ↓
复杂非凸模型出现横跨整个模型的长连线
```

### 修复方案

`C2` 和 `C4` 全部替换为 `CGAL::convex_hull_3`：

```cpp
extern "C" LIBHGP_EXPORT void HGP_3D_Convex_Hulls_C2(
    const Vector3d1 & vec,
    Vector3d1 & hull_points,
    Vector1i1 & hulls_surface_0,
    Vector1i1 & hulls_surface_1,
    Vector1i1 & hulls_surface_2)
{
    std::vector<Point_3> points;
    for (int i = 0; i < (int)vec.size(); i++)
        points.push_back(Point_3(vec[i][0], vec[i][1], vec[i][2]));

    if (points.size() <= 3) { hull_points = vec; return; }

    // CGAL 计算凸包，保证完整封闭 + 法向一致
    Polyhedron_3 poly;
    CGAL::convex_hull_3(points.begin(), points.end(), poly);

    std::map<Polyhedron_3::Vertex_const_handle, int> vertex_index_map;
    int idx = 0;
    for (auto vit = poly.vertices_begin(); vit != poly.vertices_end(); ++vit, ++idx) {
        Point_3 p = vit->point();
        hull_points.push_back(Vector3d(
            CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())));
        vertex_index_map[vit] = idx;
    }

    for (auto fit = poly.facets_begin(); fit != poly.facets_end(); ++fit) {
        auto hc = fit->facet_begin();
        int a = vertex_index_map[hc->vertex()]; ++hc;
        int b = vertex_index_map[hc->vertex()]; ++hc;
        int c = vertex_index_map[hc->vertex()];
        hulls_surface_0.push_back(a);
        hulls_surface_1.push_back(b);
        hulls_surface_2.push_back(c);
    }
}
```

CGAL `Polyhedron_3` 基于半边数据结构，天然保证：面片完整封闭、法向统一朝外、严格流形拓扑、全程 `double` 精度。

同步修改 `app.py` 凸包接口：`save_mesh(..., repair=True)` → `repair=False`（CGAL 已保证质量，无需后处理修复）。

---

## Bug 2：测地线选点强制吸附到最近顶点

### 问题现象

计算两点之间的最短测地路径时，无论用户点击网格哪个位置，起止点都自动跳到最近的网格顶点，无法落在面片内部任意位置。

### 根本原因

这是对 Bug 1 的错误补丁。Bug 1 导致凸包网格不封闭，CGAL 测地线在不封闭网格上崩溃，误判为"CGAL 测地线要求点必须在顶点上"，于是前端加了遍历最近顶点的逻辑：

```javascript
// 错误补丁：强制吸附到最近顶点
let closestVert = null, minDist = Infinity;
const posAttr = currentMesh.geometry.attributes.position;
for (let i = 0; i < posAttr.count; i++) {
    const vx = posAttr.getX(i), vy = posAttr.getY(i), vz = posAttr.getZ(i);
    const d = (vx - pt.x)**2 + (vy - pt.y)**2 + (vz - pt.z)**2;
    if (d < minDist) { minDist = d; closestVert = [vx, vy, vz]; }
}
geodesicSource = closestVert;
```

### 修复方案

删除 `closestVert` 遍历逻辑，直接使用射线与网格的精确交点：

```javascript
// 修复后：使用精确交点
const pt = hits[0].point;
geodesicSource = [pt.x, pt.y, pt.z];
```

CGAL 的 `HGP_Shortest_Geodesic_Path_C3` 内部通过 AABB 树将任意空间点投影到最近面片，不需要前端预处理。

---

## Bug 3：切片轮廓强制拼接导致长连线

### 问题现象

对复杂非凸模型（如怪物角色、带手臂腿部的模型）执行切片时，切片轮廓线出现横跨整个模型的长斜线。

### 根本原因

这也是对 Bug 1 的错误补丁。Bug 1 导致凸包切片产生异常断开的轮廓段，误判为"需要拼接多段"，后端加入了 `join_contour_segments` 贪心拼接函数：

```python
# 错误补丁：贪心拼接所有轮廓段
def join_contour_segments(segments):
    remaining = [list(seg) for seg in segments]
    result = remaining.pop(0)
    while remaining:
        tail = result[-1]
        # 找最近的段首尾并拼接
        ...
    return result
```

对凸包有效，但对复杂非凸模型，手臂、腿等本来独立的轮廓段被强行拼接，产生长连线。

### 修复方案

删除 `join_contour_segments`，切片接口改用 `offsetses`（按切平面分组），每段独立渲染：

```python
@app.post("/api/slicer")
def slicer(req: SlicerRequest):
    mesh_path = get_mesh_path(req.mesh_id, ext="off")
    offsetses, offsets = hgp_py.HGP_Slicer_Mesh(
        mesh_path, req.plane_normal, req.plane_ds
    )
    return {
        "slices": [
            {
                "contours": [
                    [[p[0], p[1], p[2]] for p in seg]
                    for seg in plane_contours
                ]
            }
            for plane_contours in offsetses
        ]
    }
```

---

## Bug 4：`save_mesh` 未同时写 OFF 文件

### 问题现象

调用切片接口 `/api/slicer` 或边界提取接口 `/api/mesh_boundary` 时，后端报文件不存在错误。

### 根本原因

`save_mesh` 只写 OBJ 文件，而 CGAL 的 `HGP_Slicer_Mesh` 和 `HGP_3D_Triangle_Mesh_Boundary_C5` 需要读 OFF 格式文件。

### 修复方案

`save_mesh` 修改为同时写 OBJ 和 OFF 两个文件。切片和边界接口调用时：

```python
# 切片接口
mesh_path = get_mesh_path(req.mesh_id, ext="off")

# 边界接口
mesh_path = get_mesh_path(req.mesh_id, ext="off")
```

---

## Bug 5：上传 OBJ 未修复非流形网格，导致 CGAL 崩溃

### 问题现象

上传复杂外部 OBJ 文件（如多边形角色模型）后，计算测地线时 CGAL 报错：

```
disconnected facet complexes at vertex 627
```

程序崩溃，返回 HTTP 500。

### 根本原因

`upload_obj` 接口的 `save_mesh` 使用 `repair=False`，不对上传的网格做流形修复。外部 OBJ 文件可能含有：
- 非流形边（一条边被 3 个以上面片共享）
- 非流形顶点（蝴蝶结顶点：顶点周围的面片不构成单一连通扇形）

CGAL 的 `Polyhedron_3` 要求严格流形网格，遇到非流形拓扑时直接崩溃。

### 修复方案

**`app.py`**：`upload_obj` 改为 `repair=True`：

```python
mesh_id = save_mesh(verts, fi0, fi1, fi2, repair=True)
```

**`repair_mesh` 新增步骤5和步骤6**：

```python
# 步骤5：删除非流形边（被3+面片共享的边）
edge_to_faces = defaultdict(list)
for idx, (a, b, c) in enumerate(zip(fixed_fi0, fixed_fi1, fixed_fi2)):
    edge_to_faces[tuple(sorted([a, b]))].append(idx)
    edge_to_faces[tuple(sorted([b, c]))].append(idx)
    edge_to_faces[tuple(sorted([a, c]))].append(idx)

bad_faces = set()
for edge, faces in edge_to_faces.items():
    if len(faces) > 2:
        for f in faces[2:]:
            bad_faces.add(f)

# 步骤6：BFS 检测蝴蝶结顶点（顶点级非流形）
# 对每个顶点，检查其相邻面片是否构成单一连通扇形
# 若有 2+ 个连通分量，只保留最大分量的面片
def get_fan_components(v, face_indices, fi0, fi1, fi2):
    neighbor_vert_to_faces = defaultdict(list)
    for f in face_indices:
        a, b, c = fi0[f], fi1[f], fi2[f]
        for u in (a, b, c):
            if u != v:
                neighbor_vert_to_faces[u].append(f)
    adj = defaultdict(set)
    for u, fs in neighbor_vert_to_faces.items():
        if len(fs) == 2:
            adj[fs[0]].add(fs[1])
            adj[fs[1]].add(fs[0])
    visited, components = set(), []
    for start in face_indices:
        if start in visited: continue
        comp, queue = set(), [start]
        while queue:
            cur = queue.pop()
            if cur in visited: continue
            visited.add(cur); comp.add(cur)
            for nb in adj[cur]:
                if nb not in visited: queue.append(nb)
    return components
```

| 检测类型 | 方法 | 处理方式 |
|---|---|---|
| 非流形边 | 无向边被 >2 个面片共享 | 保留前2个，删除多余面片 |
| 蝴蝶结顶点 | BFS 连通分量数 >1 | 保留最大分量，删除其余 |

---

## Bug 6：测地线失败时前端崩溃，报 `Cannot read properties of undefined`

### 问题现象

选取多部件模型的跨部件两点计算测地线时，前端控制台报：

```
Cannot read properties of undefined (reading 'map')
```

页面不显示任何有意义的错误信息。

### 根本原因

后端返回 HTTP 422 时，响应体是 `{ "detail": "两点不连通..." }`，没有 `path` 字段。但前端代码未检查 `res.ok`，直接执行 `data.path.map(...)`，`data.path` 为 `undefined` 导致崩溃：

```javascript
// 错误代码
const data = await res.json();
const pts = data.path.map(p => new THREE.Vector3(p[0], p[1], p[2])); // ← 崩溃
```

### 修复方案

在 `res.json()` 之后、`data.path.map` 之前加 `res.ok` 检查：

```javascript
const data = await res.json();

// 检查后端返回的错误
if (!res.ok) {
    status('错误：' + (data.detail || '测地线计算失败'));
    return;
}

const pts = data.path.map(p => new THREE.Vector3(p[0], p[1], p[2]));
```

同时在后端 `geodesic_path` 接口加检查：

```python
@app.post("/api/geodesic_path")
def geodesic_path(req: GeodesicRequest):
    mesh_path = get_mesh_path(req.mesh_id)
    path_points = hgp_py.HGP_Shortest_Geodesic_Path_C3(
        mesh_path, req.source, req.target
    )
    if len(path_points) < 2:
        raise HTTPException(
            status_code=422,
            detail="两点不连通：source 和 target 必须在同一连通网格上。"
                   "该模型可能是多部件拼合，请选择同一部件上的两点。"
        )
    dist = hgp_py.HGP_Geodesic_Distance(mesh_path, req.source, req.target)
    if dist < 0:
        raise HTTPException(status_code=422, detail="测地线计算失败：两点不在同一连通网格上。")
    return {"path": [[p[0], p[1], p[2]] for p in path_points], "distance": dist}
```

---

## Bug 7：切片接口用 `offsets` 导致轮廓展平、切平面归属丢失

### 问题现象

- cube.obj 沿 Z 轴切 3 刀，返回 8 个 slice 而不是 3 个
- 每个切面的轮廓只有部分点，无法构成完整矩形

### 根本原因

`HGP_Slicer_Mesh` 返回两个变量：

| 变量 | 结构 | 含义 |
|---|---|---|
| `offsetses` | `Vector3d3`：`list[list[list[point]]]` | 按切平面分组，每个切平面含若干轮廓段 |
| `offsets` | `Vector3d2`：`list[list[point]]` | 所有切平面所有段的展平列表 |

原始代码错误地使用了 `offsets`：

```python
# 错误代码
return {
    "slices": [
        {"contour": [[p[0], p[1], p[2]] for p in contour]}
        for contour in offsets   # ← 展平列表，切平面归属丢失
    ]
}
```

N 个切平面若每平面有 2 段，`offsets` 长度就是 2N，被当成 2N 个独立切平面返回。

### 修复方案

**后端**：改用 `offsetses`，字段名 `contour` → `contours[]`：

```python
return {
    "slices": [
        {
            "contours": [
                [[p[0], p[1], p[2]] for p in seg]
                for seg in plane_contours
            ]
        }
        for plane_contours in offsetses
    ]
}
```

**前端**：适配新的 `contours` 数组字段：

```javascript
// 修复前
data.slices.forEach((s, idx) => {
    const pts = s.contour.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    ...
});

// 修复后
data.slices.forEach((s, idx) => {
    const color = colors[idx % colors.length];
    s.contours.forEach(contour => {
        const pts = contour.map(p => new THREE.Vector3(p[0], p[1], p[2]));
        if (!pts.length) return;
        scene.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(pts),
            new THREE.LineBasicMaterial({ color })
        ));
    });
});
```

---

## Bug 8：切片 `plane_ds` 符号错误，切割平面全在模型范围外

### 问题现象

切片接口返回 `offsetses` 每个平面均为 0 段：

```
offsetses 长度: 5, 各平面段数: [0, 0, 0, 0, 0]
offsets 长度: 0
```

不渲染任何切片轮廓。

### 根本原因

前端计算切割平面的 `ds` 时多加了一个负号：

```javascript
// 错误代码
const ds = Array.from({ length: n }, (_, i) =>
    -(vMin + (vMax - vMin) * (i + 1) / (n + 1))  // ← 多余的负号
);
```

C++ 内部平面方程为：`normal·p + (-plane_d) = 0`，即切割坐标 = `plane_d`。

cube.obj 的 Z 范围是 `[0, 1]`，加了负号后 `ds = [-0.167, -0.333, ...]`，切割坐标全是负数，全在模型外面，切不到任何面。

### 修复方案

去掉多余负号：

```javascript
// 修复后
const ds = Array.from({ length: n }, (_, i) =>
    vMin + (vMax - vMin) * (i + 1) / (n + 1)
);
```

cube.obj 切 5 刀的结果：`ds = [0.167, 0.333, 0.5, 0.667, 0.833]`，全在 `[0,1]` 范围内，正确穿过模型。

---

## Bug 9：多部件模型测地线返回 -1，无友好提示

### 问题现象

对多部件角色模型（如 Tails 角色，身体各部件为独立网格）选取跨部件两点计算测地线时，结果返回 -1，前端无任何有意义的提示。

### 根本原因

测地线算法要求 source 和 target 在**同一连通的流形网格**上。多部件模型的各部件是独立的连通分量，网格之间没有共享边/顶点，CGAL 找不到路径，返回空路径或 -1。

```
头部网格  ←── 独立连通分量 A
身体网格  ←── 独立连通分量 B（与 A 无共享边）
→ 测地线无解，返回 -1
```

### 修复方案

后端加检查，返回 HTTP 422 和中文错误说明（见 Bug 6 修复方案中的后端代码）。

前端已通过 Bug 6 的 `res.ok` 检查正确显示错误信息。

**根本解决方案**（如需跨部件测地线）：需在建模软件中将所有部件 Merge 成一个连通网格后导出 OBJ，纯算法层面无法在不连通的网格之间计算测地距离。

---

## 修复效果对比

| 功能 | 修复前 | 修复后 |
|---|---|---|
| 凸包网格完整性 | 随机缺失 2～4 个面片 | 数学上严格完整 |
| 测地线起止点 | 只能在顶点上 | 可在面片内任意位置 |
| 凸包切片轮廓 | 出现长连线 | 正确的闭合环 |
| 复杂模型切片 | 强行拼接导致错误 | 独立弧段正确渲染 |
| 切片平面数量 | 返回 2N 个（每平面2段展平） | 正确返回 N 个切平面 |
| 切割位置 | 全在模型外（符号错误） | 均匀分布在模型内 |
| 外部OBJ测地线 | CGAL 崩溃 HTTP 500 | 正常计算或友好错误提示 |
| 跨部件测地线 | 无任何提示，前端崩溃 | 显示"两点不连通"提示 |
| 代码复杂度 | 多层补丁叠加 | 删除所有补丁，逻辑简洁 |

---

## 经验教训

### 1. 表象问题不等于根本原因

测地线崩溃（Bug 2）和切片断裂（Bug 3）看起来是两个独立问题，实际上同源于凸包缺失面片（Bug 1）。沿着表象打补丁只会让代码越来越复杂。

### 2. 几何算法的错误具有传播性

底层数据质量问题（不封闭网格）会向上传播，在所有依赖该数据的算法中都表现为"奇怪的错误"。遇到多个模块同时出错时，优先检查数据来源。

### 3. 先验证数据，再调试算法

遇到几何算法异常，第一步应验证输入数据的拓扑正确性（欧拉公式、边界边检测），而不是直接修改算法逻辑。

```python
# 快速检测网格是否流形（边界边数 = 0 才是封闭流形）
from collections import defaultdict
edge_count = defaultdict(int)
for a, b, c in zip(fi0, fi1, fi2):
    for e in [(min(a,b),max(a,b)), (min(b,c),max(b,c)), (min(a,c),max(a,c))]:
        edge_count[e] += 1
boundary_edges = [e for e, cnt in edge_count.items() if cnt == 1]
print(f"边界边数: {len(boundary_edges)}")  # 0 = 封闭流形
```

### 4. float 精度陷阱

几何算法中 `double→float` 的截断会改变点的空间关系，触发算法边界情况。涉及坐标计算时，全程使用 `double`。

### 5. 前端要总是检查 HTTP 状态码

任何 `fetch` 调用后都必须检查 `res.ok`，再访问响应字段，否则非 2xx 响应会导致 JavaScript 崩溃：

```javascript
const data = await res.json();
if (!res.ok) { status('错误：' + (data.detail || '未知错误')); return; }
// 只有在这里才能安全访问 data.xxx
```

### 6. repair 是补丁，不是根治

能在生成阶段保证数据质量，就不要依赖后处理修复。`repair_mesh` 能修复法向冲突、非流形边等问题，但无法补全根本没有生成的面片。

---

*文档生成时间：2026-03-31*