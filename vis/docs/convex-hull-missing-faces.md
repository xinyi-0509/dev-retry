# 凸包网格缺失面片问题排查与修复

**问题模块**：`HGP_3D_Convex_Hulls_C2` / `HGP_3D_Convex_Hulls_C4`
**涉及文件**：`hull3D.cpp`、`vis/backend/app.py`
**日期**：2026-03-31

---

## 一、问题现象

使用 `/api/convex_hull_3d` 接口生成随机点的 3D 凸包时，部分情况下网格**缺少若干面片**，在可视化工具中可以看到明显的"破洞"（如图中标注的 1、2、3 处）。

该问题具有随机性：大多数点集生成正常，但特定分布的点集会稳定复现缺失 2～4 个面片的情况。

---

## 二、为什么缺失面片：两次调用 `NewtonApple_hull_3D`

### 原始代码逻辑

```cpp
// 第一次调用
int nx = de_duplicateR3(pts, outx, pts2);
int ts = NewtonApple_hull_3D(pts2, tris);   // ← 第一次，结果正确

// 把第一次结果的面片顶点展开成新点列表
for (int i = 0; i < (int)tris.size(); i++)
{
    pts.push_back(pts2[tris[i].a]);
    pts.push_back(pts2[tris[i].b]);
    pts.push_back(pts2[tris[i].c]);
}

// 清空，对展开后的点再次去重
pts2.clear();
outx.clear();
tris.clear();
nx = de_duplicateR3(pts, outx, pts2);       // ← 对展开点去重

// 第二次调用 ← bug 根源
ts = NewtonApple_hull_3D(pts2, tris);       // ← 结果不完整
```

### 为什么要两次调用

第一次调用 `NewtonApple_hull_3D` 存在两个已知问题：

1. **法线方向不统一**：输出的三角面片顶点顺序没有保证统一朝外，部分面片法向朝内
2. **重复顶点干扰**：输入点集中的重复或近似点可能导致退化面片

原始设计意图是：**将第一次凸包结果的顶点重新展开、去重后再算一次**，希望通过第二次调用得到更干净的结果，同时隐式地修正法向问题。

### 为什么第二次调用反而导致缺失面片

第二次调用的输入 `pts2` 是对"面片顶点展开列表"去重后的结果：

```
原始点集（N个点）
    ↓ de_duplicateR3
去重点集（M个点，M≤N）
    ↓ NewtonApple_hull_3D（第一次）
凸包面片（F个三角形，3F个顶点引用）
    ↓ 展开为点列表
3F 个点（含大量重复）
    ↓ de_duplicateR3（第二次去重）
新点集（K个点）← 与原始去重点集不同！
    ↓ NewtonApple_hull_3D（第二次）
不完整的凸包面片 ← 缺失面片
```

关键问题在于：**第二次 `de_duplicateR3` 的输入是面片顶点的展开列表，而不是原始点集**。这个展开列表的点分布与原始点集存在微小的浮点差异（因为 `R3` 结构体使用 `float` 而非 `double` 存储坐标），导致去重结果不一致，进而使 `NewtonApple_hull_3D` 在第二次调用时面对一个"不标准"的点集，在某些几何配置下输出不完整的凸包。

### 精度损失的具体路径

```cpp
// 原始输入是 double
pt.r = (float)vec[i][0];   // ← double → float，精度损失
pt.c = (float)vec[i][1];
pt.z = (float)vec[i][2];
```

`R3` 结构体用 `float` 存储坐标，而 `Vector3d` 是 `double`。每次从 `pts2` 读出再写入 `pts` 时，都经历了一次 `float→double→float` 的精度往返，累积误差改变了点的空间分布，触发了 `NewtonApple_hull_3D` 的边界情况。

---

## 三、为什么 `repair_mesh` 无法解决这个问题

`repair_mesh` 具备以下能力：

| 功能 | 能力 |
|---|---|
| 合并重复顶点 | ✅ |
| 删除退化面片 | ✅ |
| 删除重复面片 | ✅ |
| 修复法向冲突 | ✅ |
| **补全缺失面片** | ❌ |

缺失面片的问题是底层算法**根本没有生成这些面片**，`repair_mesh` 拿不到不存在的数据，无从修复。这类问题必须在生成阶段解决，而不能在修复阶段弥补。

---

## 四、为什么不用 `scipy.ConvexHull` 作为替代

`scipy.spatial.ConvexHull`（基于 Qhull）可以在 Python 层正确生成凸包，但引入了外部依赖，违背了本库"使用自有算法"的设计初衷。因此最终选择在 C++ 底层修复。

---

## 五、最终解决方案：换用 CGAL `convex_hull_3`

### 为什么选 CGAL

`C1` 函数已经证明了 CGAL 的可靠性，它用的就是 `CGAL::convex_hull_3`：

```cpp
// C1 的实现（已验证正确）
Polyhedron_3 poly;
CGAL::convex_hull_3(points.begin(), points.end(), poly);
```

CGAL 的 `Polyhedron_3` 基于**半边数据结构**，天然保证：

| 保证项 | 说明 |
|---|---|
| 完整封闭 | 数学上严格的凸包，不会漏面片 |
| 法向统一朝外 | 半边结构保证面片绕向一致 |
| 流形拓扑 | 每条边恰好被两个面片共享 |
| 索引匹配 | 通过顶点句柄映射，精确对应 |
| 可验证 | `poly.is_valid()` / `poly.is_closed()` |

### 修复后的 `HGP_3D_Convex_Hulls_C2`

```cpp
extern "C" LIBHGP_EXPORT void HGP_3D_Convex_Hulls_C2(
    const Vector3d1 & vec,
    Vector3d1 & hull_points,
    Vector1i1 & hulls_surface_0,
    Vector1i1 & hulls_surface_1,
    Vector1i1 & hulls_surface_2)
{
    // 1. 构建 CGAL 点集
    std::vector<Point_3> points;
    for (int i = 0; i < (int)vec.size(); i++)
        points.push_back(Point_3(vec[i][0], vec[i][1], vec[i][2]));

    if (points.size() <= 3)
    {
        hull_points = vec;
        return;
    }

    // 2. CGAL 计算凸包
    Polyhedron_3 poly;
    CGAL::convex_hull_3(points.begin(), points.end(), poly);

    // 3. 导出顶点，建立句柄→索引映射
    std::map<Polyhedron_3::Vertex_const_handle, int> vertex_index_map;
    int idx = 0;
    for (auto vit = poly.vertices_begin();
         vit != poly.vertices_end(); ++vit, ++idx)
    {
        Point_3 p = vit->point();
        hull_points.push_back(Vector3d(
            CGAL::to_double(p.x()),
            CGAL::to_double(p.y()),
            CGAL::to_double(p.z())
        ));
        vertex_index_map[vit] = idx;
    }

    // 4. 导出面片索引
    for (auto fit = poly.facets_begin();
         fit != poly.facets_end(); ++fit)
    {
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

### 修复后的 `HGP_3D_Convex_Hulls_C4`

`C4` 在 `C2` 基础上额外输出每个面片的平面参数：

```cpp
extern "C" LIBHGP_EXPORT void HGP_3D_Convex_Hulls_C4(
    const Vector3d1 & vec,
    Vector3d1 & hull_points,
    Vector1i1 & hulls_surface_0,
    Vector1i1 & hulls_surface_1,
    Vector1i1 & hulls_surface_2,
    Vector3d1 & plane_p,
    Vector3d1 & plane_n)
{
    std::vector<Point_3> points;
    for (int i = 0; i < (int)vec.size(); i++)
        points.push_back(Point_3(vec[i][0], vec[i][1], vec[i][2]));

    if (points.size() <= 3)
    {
        hull_points = vec;
        return;
    }

    Polyhedron_3 poly;
    CGAL::convex_hull_3(points.begin(), points.end(), poly);

    std::map<Polyhedron_3::Vertex_const_handle, int> vertex_index_map;
    int idx = 0;
    for (auto vit = poly.vertices_begin();
         vit != poly.vertices_end(); ++vit, ++idx)
    {
        Point_3 p = vit->point();
        hull_points.push_back(Vector3d(
            CGAL::to_double(p.x()),
            CGAL::to_double(p.y()),
            CGAL::to_double(p.z())
        ));
        vertex_index_map[vit] = idx;
    }

    for (auto fit = poly.facets_begin();
         fit != poly.facets_end(); ++fit)
    {
        auto hc = fit->facet_begin();
        int a = vertex_index_map[hc->vertex()]; ++hc;
        int b = vertex_index_map[hc->vertex()]; ++hc;
        int c = vertex_index_map[hc->vertex()];

        hulls_surface_0.push_back(a);
        hulls_surface_1.push_back(b);
        hulls_surface_2.push_back(c);

        // 平面参数：从半边直接取顶点坐标
        auto hc2 = fit->facet_begin();
        Point_3 pa = hc2->vertex()->point(); ++hc2;
        Point_3 pb = hc2->vertex()->point(); ++hc2;
        Point_3 pc = hc2->vertex()->point();

        Plane_3 plane(pa, pb, pc);

        Point_3 center(
            (CGAL::to_double(pa.x()) + CGAL::to_double(pb.x()) + CGAL::to_double(pc.x())) / 3.0,
            (CGAL::to_double(pa.y()) + CGAL::to_double(pb.y()) + CGAL::to_double(pc.y())) / 3.0,
            (CGAL::to_double(pa.z()) + CGAL::to_double(pb.z()) + CGAL::to_double(pc.z())) / 3.0
        );
        plane_p.push_back(PointVector3d(center));

        Vector_3 n = plane.orthogonal_vector();
        plane_n.push_back(Vector3d(
            CGAL::to_double(n.x()),
            CGAL::to_double(n.y()),
            CGAL::to_double(n.z())
        ));
    }
}
```

---

## 六、修复后 Python 层的简化

修复前需要 `repair=True` 来弥补底层问题：

```python
# 修复前：依赖 repair_mesh 修正法向冲突
mesh_id = save_mesh(hull_verts, fi0, fi1, fi2, repair=True)
```

修复后 CGAL 已保证输出质量，`repair=False` 即可：

```python
# 修复后：CGAL 已保证完整封闭 + 法向一致
mesh_id = save_mesh(hull_verts, fi0, fi1, fi2, repair=False)
```

---

## 七、问题对比总结

| 对比项 | 原始实现（NewtonApple × 2） | 修复后（CGAL） |
|---|---|---|
| 算法 | 自定义 NewtonApple_hull_3D | CGAL::convex_hull_3 |
| 面片完整性 | ❌ 部分情况缺失 2～4 个面片 | ✅ 数学上严格完整 |
| 法向一致性 | ❌ 不保证，依赖二次调用修正 | ✅ 半边结构天然保证 |
| 流形拓扑 | ❌ 不保证 | ✅ 可通过 is_valid() 验证 |
| 精度 | ❌ double→float→double 损失 | ✅ 全程 double |
| 代码复杂度 | 高（两次调用 + 展开 + 去重） | 低（一次调用 + 遍历导出） |

---

## 八、经验总结

1. **不要对凸包结果做二次凸包**：凸包的顶点展开后再算凸包，点集已经改变，结果不可预测
2. **float 精度陷阱**：几何算法中 `double→float` 的截断会改变点的空间关系，触发算法边界情况
3. **优先用经过严格验证的库**：CGAL 的几何算法有数学证明，自定义实现需要更多边界情况测试
4. **repair 是补丁，不是根治**：能在生成阶段保证质量，就不要依赖后处理修复