# 面向数字制造的 CSG 功能设计与实现

> **文档版本**：v1.0 · 2026-04-01  
> **所属项目**：LibHGP 几何算法可视化系统  
> **涉及模块**：`hgp_py.cpp`（C++ / pybind11）、`vis/backend/app.py`（FastAPI 后端）、`vis/backend/web/index.html`（Three.js 前端）

---

## 1 背景与需求

LibHGP 是一个以 CGAL 为底层的几何处理算法库，原有功能集中于网格分析类算法（曲率、测地线、切片、平滑、细分等）。在面向**数字制造**场景（如 CAD/CAM、三维打印前处理、零件加工仿真）的扩展需求中，需要引入**构造实体几何（Constructive Solid Geometry，CSG）**能力，使用户能够通过布尔运算对三角网格模型进行直接的几何编辑。

本次扩展的核心目标为：

1. 在 C++ 库层新增 **CSG 基元生成**与**布尔运算**函数，并通过 pybind11 暴露给 Python；
2. 在后端服务层基于上述接口实现**圆柱孔打孔**功能（Difference 布尔）；
3. 在前端实现**交互式法面拾取**与**实时预览**，提供接近工业软件的操作体验。

---

## 2 底层算法层：CSG 函数的 C++ 实现与 pybind11 绑定

### 2.1 新增 C++ 函数接口

在 `libhgp.h` 中新增两类函数，均以 `extern "C" LIBHGP_EXPORT` 方式导出，保证跨编译器 ABI 兼容。

#### 2.1.1 基元生成函数

| 函数签名 | 说明 |
|---|---|
| `HGP_Mesh_Make_Box(cx,cy,cz, wx,wy,wz, vecs,fi0,fi1,fi2)` | 生成以 `(cx,cy,cz)` 为中心、半轴为 `wx/wy/wz` 的长方体 |
| `HGP_Mesh_Make_Cylinder(cx,cy,cz, radius,height,segments, vecs,fi0,fi1,fi2)` | 生成以 `(cx,cy,cz)` 为**底面中心**、沿 +Z 轴延伸 `height` 的圆柱，环向分段数为 `segments` |
| `HGP_Mesh_Make_Sphere(cx,cy,cz, radius,rings,segments, vecs,fi0,fi1,fi2)` | 生成 UV 球体 |
| `HGP_Mesh_Make_Cone(cx,cy,cz, radius,height,segments, vecs,fi0,fi1,fi2)` | 生成圆锥，顶点在 `(cx, cy, cz+height)` |

所有基元均输出**三角化封闭流形网格**（顶点表 `Vector3d1`，面索引表 `Vector1i1 fi0/fi1/fi2`），满足 CGAL 布尔运算对输入网格的严格要求（封闭、可定向、无自交）。

#### 2.1.2 布尔运算函数

```cpp
// 并集：result = A ∪ B
extern "C" LIBHGP_EXPORT bool HGP_Mesh_CSG_Union(
    const Vector3d1& vecs_a, const Vector1i1& fi0_a,
    const Vector1i1& fi1_a,  const Vector1i1& fi2_a,
    const Vector3d1& vecs_b, const Vector1i1& fi0_b,
    const Vector1i1& fi1_b,  const Vector1i1& fi2_b,
    Vector3d1& vecs_out, Vector1i1& fi0_out,
    Vector1i1& fi1_out,  Vector1i1& fi2_out);

// 差集：result = A − B
extern "C" LIBHGP_EXPORT bool HGP_Mesh_CSG_Difference(/* 同上 */);

// 交集：result = A ∩ B
extern "C" LIBHGP_EXPORT bool HGP_Mesh_CSG_Intersection(/* 同上 */);
```

三个函数均返回 `bool` 表示运算是否成功，失败原因通常为两网格不满足流形条件或交叠区域退化。

### 2.2 pybind11 绑定实现

在 `hgp_py.cpp` 的 `PYBIND11_MODULE(hgp_py, m)` 块中，利用已有的辅助转换函数（`to_vec3d1`、`from_vec3d1` 等）将 C++ 的 `Vector3d1 / Vector1i1` 类型与 Python 的 `list[list[float]] / list[int]` 互转。

**基元生成绑定示例（以圆柱为例）：**

```cpp
m.def("HGP_Mesh_Make_Cylinder",
    [](double cx, double cy, double cz,
       double radius, double height, int segments)
       -> std::tuple<std::vector<std::vector<double>>,
                     std::vector<int>, std::vector<int>, std::vector<int>> {
        Vector3d1 vecs; Vector1i1 fi0, fi1, fi2;
        HGP_Mesh_Make_Cylinder(cx, cy, cz, radius, height, segments,
                               vecs, fi0, fi1, fi2);
        return {from_vec3d1(vecs), fi0, fi1, fi2};
    },
    py::arg("cx"), py::arg("cy"), py::arg("cz"),
    py::arg("radius"), py::arg("height"), py::arg("segments"),
    "生成圆柱网格，底面中心在 (cx,cy,cz)，沿 +Z 延伸 height，返回 (verts, fi0, fi1, fi2)。");
```

**布尔运算绑定示例（以差集为例）：**

```cpp
m.def("HGP_Mesh_CSG_Difference",
    [](const std::vector<std::vector<double>>& va,
       const std::vector<int>& f0a, const std::vector<int>& f1a,
       const std::vector<int>& f2a,
       const std::vector<std::vector<double>>& vb,
       const std::vector<int>& f0b, const std::vector<int>& f1b,
       const std::vector<int>& f2b)
       -> std::tuple<bool,
                     std::vector<std::vector<double>>,  
                     std::vector<int>, std::vector<int>, std::vector<int>> {
        Vector3d1 vout; Vector1i1 fo0, fo1, fo2;
        bool ok = HGP_Mesh_CSG_Difference(
            to_vec3d1(va), f0a, f1a, f2a,
            to_vec3d1(vb), f0b, f1b, f2b,
            vout, fo0, fo1, fo2);
        return {ok, from_vec3d1(vout), fo0, fo1, fo2};
    }, /* py::arg ... */
    "差集布尔运算 A−B，返回 (success, verts, fi0, fi1, fi2)。");
```

绑定设计遵循**只暴露纯数据结构**的原则：输入输出均为 Python 原生列表，无需在 Python 侧感知 C++ 自定义类型，降低使用门槛，也便于与 NumPy 等生态工具互操作。

---

## 3 后端服务层：打孔算法设计

### 3.1 总体架构

后端采用 **FastAPI** 框架，通过三个 REST 接口向前端提供 CSG 服务：

| 路由 | 方法 | 功能 |
|---|---|---|
| `/api/csg/preview_cylinder` | POST | 生成并返回工具圆柱网格（供前端预览，不修改主体） |
| `/api/csg/hole` | POST | 执行圆柱孔打孔（Difference 布尔），返回结果网格 |
| `/api/csg/boolean` | POST | 通用两网格布尔运算（Union / Difference / Intersection） |

### 3.2 请求数据模型

```python
class CSGHoleRequest(BaseModel):
    mesh_id: str          # 主体网格标识符
    center:  list[float]  # 拾取点世界坐标 [x, y, z]
    normal:  list[float]  # 拾取面的单位法向量 [nx, ny, nz]
    radius:  float        # 圆柱孔半径
    depth:   float        # 圆柱孔深度

class CSGPreviewRequest(BaseModel):
    center: list[float]
    normal: list[float]
    radius: float
    depth:  float
```

### 3.3 关键算法：轴向对齐圆柱生成（`_make_oriented_cylinder`）

这是本次实现中技术难度最高的环节。目标是生成**以拾取点为轴向中点**、**沿任意法线方向**的圆柱网格。

#### 3.3.1 设计约束

`HGP_Mesh_Make_Cylinder(cx, cy, cz, r, h, n)` 的语义是：生成**底面中心**在 `(cx,cy,cz)`、沿 **+Z 轴**向上延伸的圆柱。直接传入拾取点坐标会导致两类错误：

- **方向错误**：圆柱始终沿 Z 轴，不跟随法线方向；
- **位置偏移**（旋转后）：若先生成带偏移位置的圆柱再旋转，`HGP_Rotation_Obj` 的旋转轴过世界**原点**而非拾取点，导致圆柱轴线在旋转后不再经过拾取点。

上述第二类错误是导致"正面打孔位置正确、侧面打孔位置偏移"现象的根本原因，其物理本质是**旋转中心与圆柱几何中心不重合**。

#### 3.3.2 正确算法：原点生成 → 旋转对齐 → 平移就位

```
步骤 1：在原点生成圆柱，令其以原点为轴向中点
        底面中心 = (0, 0, -h/2)，顶面中心 = (0, 0, +h/2)
        → HGP_Mesh_Make_Cylinder(0, 0, -h/2, r, h, 48)

步骤 2：计算从 +Z 轴旋转到目标法线 n 的旋转变换
        旋转轴 k = Z × n / |Z × n|
        旋转角 θ = arccos(Z · n)
        → HGP_Rotation_Obj(tmp_obj, θ, k)
        （此时旋转中心 = 原点 = 圆柱轴向中点，旋转不引入位移误差）

步骤 3：将所有顶点平移到拾取点 center
        v_final = v_rotated + center
```

```python
def _make_oriented_cylinder(center, normal_normalized,
                            radius, height, segments=48):
    ax, ay, az = normal_normalized
    half_h = height / 2.0

    # 步骤 1：在原点生成圆柱，轴向中点在原点
    cyl_verts, cyl_fi0, cyl_fi1, cyl_fi2 = hgp_py.HGP_Mesh_Make_Cylinder(
        0.0, 0.0, -half_h, radius, height, segments)

    # 步骤 2：旋转对齐法线（绕过原点的轴旋转）
    z_axis = [0.0, 0.0, 1.0]
    dot = ax*z_axis[0] + ay*z_axis[1] + az*z_axis[2]
    if abs(dot - 1.0) > 1e-6:          # 法线不是 +Z 才需要旋转
        rx = z_axis[1]*az - z_axis[2]*ay
        ry = z_axis[2]*ax - z_axis[0]*az
        rz = z_axis[0]*ay - z_axis[1]*ax
        rl = math.sqrt(rx*rx + ry*ry + rz*rz)
        if rl > 1e-9:
            angle = math.acos(max(-1.0, min(1.0, dot)))
            rot_axis = [rx/rl, ry/rl, rz/rl]
            tmp_id, _, _, _, _ = save_mesh(
                cyl_verts, cyl_fi0, cyl_fi1, cyl_fi2, repair=False)
            tmp_obj = get_mesh_path(tmp_id, ext="obj")
            hgp_py.HGP_Rotation_Obj(tmp_obj, angle, rot_axis)
            cyl_verts, cyl_fi0, cyl_fi1, cyl_fi2 = 
                _load_mesh_from_obj(tmp_obj)

    # 步骤 3：平移到拾取点
    cx, cy, cz = center
    cyl_verts = [[v[0]+cx, v[1]+cy, v[2]+cz] for v in cyl_verts]
    return cyl_verts, cyl_fi0, cyl_fi1, cyl_fi2
```

**正确性验证**：对任意法线方向 $
abla$，旋转变换 $R$ 满足 $R\,	extbf{e}_z = \nabla$，且旋转中心为原点（即圆柱轴向中点），因此旋转后轴向中点仍在原点；最终平移 `+ center` 使轴向中点精确落在拾取点处，与法线方向无关。

#### 3.3.3 深度裕量设计

在打孔接口中，工具圆柱深度取 `depth * 1.05`（5% 裕量）：

```python
cyl_verts, ... = _make_oriented_cylinder(
    req.center, [ax, ay, az], req.radius, req.depth * 1.05)
```

该裕量确保圆柱两端面略超出主体网格表面，避免 CGAL 布尔运算在共面情况下产生退化结果。预览接口使用精确深度 `depth`，不添加裕量，以准确反映用户期望的孔深。

### 3.4 布尔运算接口实现

```python
def _do_boolean(verts_a, fi0_a, fi1_a, fi2_a,
                verts_b, fi0_b, fi1_b, fi2_b,
                operation: str):
    if operation == "union":
        ok, verts_out, fi0_out, fi1_out, fi2_out = \
            hgp_py.HGP_Mesh_CSG_Union(
                verts_a, fi0_a, fi1_a, fi2_a,
                verts_b, fi0_b, fi1_b, fi2_b)
    elif operation == "difference":
        ok, verts_out, fi0_out, fi1_out, fi2_out = \
            hgp_py.HGP_Mesh_CSG_Difference(...)
    elif operation == "intersection":
        ok, verts_out, fi0_out, fi1_out, fi2_out = \
            hgp_py.HGP_Mesh_CSG_Intersection(...)
    else:
        raise HTTPException(400, ...)

    if not ok or not verts_out:
        raise HTTPException(422,
            "布尔运算失败或结果为空，"
            "请确认两个网格均为封闭流形且存在交叠区域。")
    return verts_out, fi0_out, fi1_out, fi2_out
```

运算完成后的结果网格通过 `save_mesh()` 持久化为 OBJ 与 OFF 双格式：OBJ 供后续加载与布尔运算使用，OFF 格式供 `HGP_Slicer_Mesh` 等需要 OFF 输入的算法调用。

### 3.5 网格预处理：居中平移

为保证 CSG 运算中拾取坐标与存储坐标系一致，上传 OBJ 文件时对网格执行 `_center_mesh()`：

```python
def _center_mesh(vertices, fi0, fi1, fi2):
    """将网格包围盒中心平移到原点，保留尺寸，不缩放。"""
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    cz = (min(zs) + max(zs)) / 2.0
    new_verts = [[v[0]-cx, v[1]-cy, v[2]-cz] for v in vertices]
    return new_verts, fi0, fi1, fi2
```

相比早期方案（等比归一化到 `[-1,1]`），纯平移方案保留了模型的真实尺寸和比例，使后续测地线计算、圆柱孔深度等需要真实度量的算法结果有意义。

---

## 4 前端交互层：法面拾取与预览

### 4.1 统一拾取状态机

前端使用单一 `pickStep` 变量管理所有拾取模式，避免多个独立事件监听器之间的状态竞争：

```
pickStep = 0  →  空闲
pickStep = 1  →  等待测地线起点
pickStep = 2  →  等待测地线终点
pickStep = 3  →  等待打孔圆心
```

### 4.2 法面法向量获取

Three.js `Raycaster` 的 `intersectObject()` 返回命中信息，其中 `hit.face.normal` 为**局部坐标系**下的面法向量，需变换到世界坐标系：

```javascript
const faceN = hit.face.normal.clone()
    .transformDirection(currentMesh.matrixWorld)
    .normalize();
```

该世界坐标系法向量直接作为 `normal` 字段发送到后端，无需额外转换。

### 4.3 实时预览圆柱

用户点击网格后，前端**立即**向 `/api/csg/preview_cylinder` 发起请求，将返回的圆柱网格以半透明橙色材质渲染，供用户确认打孔位置与方向：

```javascript
fetch(`${API}/csg/preview_cylinder`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
        center: holePick.center,
        normal: holePick.normal,
        radius, depth
    }),
}).then(r => r.json()).then(data => {
    const geo = buildGeometry(data);
    holePreviewMesh = new THREE.Mesh(geo,
        new THREE.MeshPhongMaterial({
            color: 0xff6600, transparent: true,
            opacity: 0.45, side: THREE.DoubleSide,
            depthWrite: false,
        }));
    scene.add(holePreviewMesh);
});
```

预览网格在执行打孔或撤销时被移除，确保场景状态一致。

### 4.4 动态标记球半径

测地线端点与打孔圆心的选取标记球（`SphereGeometry`）半径根据**命中面片的最长边长度**动态计算：

```javascript
function faceSize(hit) {
    const pos = hit.object.geometry.attributes.position;
    const toV = (i) => new THREE.Vector3()
        .fromBufferAttribute(pos, i)
        .applyMatrix4(hit.object.matrixWorld);
    const { a, b, c } = hit.face;
    const va = toV(a), vb = toV(b), vc = toV(c);
    const longestEdge = Math.max(
        va.distanceTo(vb),
        vb.distanceTo(vc),
        vc.distanceTo(va));
    return Math.max(longestEdge * 0.1, 0.005);
}
```

该设计使标记球大小随网格密度自适应，在粗粒度网格与精细网格上均保持良好的视觉可读性。

### 4.5 撤销机制

打孔前记录主体网格的 `prevMeshId`；操作完成后通过 `undoCSGHole()` 调用 `/api/mesh/reload` 接口恢复：

```javascript
async function undoCSGHole() {
    if (!prevMeshId) return;
    const resp = await fetch(`${API}/mesh/reload`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mesh_id: prevMeshId }),
    });
    const data = await resp.json();
    applyMeshData(data);
    currentMeshId = prevMeshId;
    prevMeshId = null;
}
```

---

## 5 调试过程与问题记录

### 问题 1：打孔位置偏移（侧面法线方向）

**现象**：点击正面（法线平行于 Z 轴）打孔位置正确；点击侧面或顶面时，预览圆柱偏离拾取点。

**根因**：旧代码将拾取点坐标直接传入 `HGP_Mesh_Make_Cylinder`，使圆柱底面落在拾取点处（而非轴向中点）。此后 `HGP_Rotation_Obj` 以世界原点为旋转中心，将整个偏置圆柱旋转，导致轴线偏移。仅当法线恰为 +Z 轴（旋转角为 0，无旋转操作）时不产生偏移，与观察结果一致。

**修复**：采用"原点生成 → 旋转对齐 → 平移就位"三步流程，旋转始终绕过轴向中点（原点）进行，彻底消除偏移。

### 问题 2：预览圆柱朝外

**现象**：修复位置问题后，圆柱从拾取点向**法线朝外**方向延伸，而非对称穿过网格。

**根因**：`HGP_Mesh_Make_Cylinder(0, 0, 0, r, h)` 以 `z=0` 为底面、向 `+z` 延伸，导致圆柱轴向中点在 `z=+h/2` 而非原点。旋转后圆柱中心偏向法线方向。

**修复**：将底面中心设为 `(0, 0, -h/2)`，使圆柱以原点为轴向中点对称分布于 `[-h/2, +h/2]`，旋转后以拾取点为中心向两侧均匀延伸。

### 问题 3：布尔运算失败（流形条件不满足）

**现象**：部分 OBJ 文件导入后执行打孔操作时，CGAL 返回失败。

**根因**：任意来源的 OBJ 文件可能包含重复顶点、退化面片、非流形边、蝴蝶结顶点等不满足 CGAL 严格流形要求的几何缺陷。

**修复**：在 `upload_obj` 接口中调用 `repair_mesh()`，依次执行六步修复流程：合并重复顶点 → 删除退化面片 → 删除重复面片 → 统一半边方向 → 删除非流形边 → 消除蝴蝶结顶点。

---

## 6 接口汇总

### 6.1 `/api/csg/preview_cylinder`

| 字段 | 类型 | 说明 |
|---|---|---|
| `center` | `[float, float, float]` | 拾取点世界坐标 |
| `normal` | `[float, float, float]` | 拾取面法向量（无需预先归一化） |
| `radius` | `float` | 圆柱半径 |
| `depth` | `float` | 圆柱深度 |

**返回**：`{ vertices: float[], faces: int[], mesh_id: str }`

### 6.2 `/api/csg/hole`

同 `preview_cylinder` 请求体，额外包含 `mesh_id`（主体网格标识）。

**返回**：布尔运算结果网格（与 `mesh_response` 统一格式）

### 6.3 `/api/csg/boolean`

| 字段 | 类型 | 说明 |
|---|---|---|
| `mesh_id_a` | `str` | 网格 A 标识 |
| `mesh_id_b` | `str` | 网格 B 标识 |
| `operation` | `str` | `"union"` / `"difference"` / `"intersection"` |

---

## 7 总结

本次扩展通过以下三层协同实现了面向数字制造的 CSG 打孔功能：

1. **C++ / pybind11 层**：新增基元生成（Box、Cylinder、Sphere、Cone）与布尔运算（Union、Difference、Intersection）共 7 个函数，全部通过 pybind11 绑定为 Python 可调用接口，严格遵循"只传递原生数据结构"的绑定原则；

2. **FastAPI 后端层**：设计 `_make_oriented_cylinder` 算法，以"原点生成 → 旋转对齐 → 平移就位"三步消除任意法线方向下的圆柱位置偏移问题；通过 `repair_mesh` 保证输入网格满足 CGAL 流形条件；

3. **Three.js 前端层**：实现基于 Raycaster 的法面拾取、实时半透明圆柱预览、动态自适应标记球半径以及单步撤销机制，提供完整的交互式操作流程。

调试过程中发现并修复的两个核心坐标错误（位置偏移与方向朝外）均源于对 `HGP_Mesh_Make_Cylinder` 底面语义的误解，最终通过数学分析确认了"旋转中心必须与圆柱几何中心重合"这一约束，并设计了正确的三步变换流程加以解决。