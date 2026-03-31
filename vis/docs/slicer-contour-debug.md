# 网格切片轮廓错误排查与修复

**问题模块**：`/api/slicer` 接口 + 前端 `runSlicer`  
**涉及文件**：`vis/backend/app.py`、`vis/backend/web/index.html`

---

## 一、问题现象

对随机生成的 3D 凸包执行网格切片时，切片轮廓线出现**跳跃、交叉、不闭合**的异常，无法正确描绘截面形状。

![切片错误示意](../../images/slicer-bug.png)

---

## 二、排查过程

### 第1步：怀疑前端点排序问题

**初步判断**：前端把切片点直接按返回顺序连线，没有排序，导致轮廓乱跳。

在前端加入最近邻排序函数 `sortContour`：

```javascript
function sortContour(pts) {
  if (pts.length < 2) return pts;
  const result = [pts[0]];
  const used   = new Array(pts.length).fill(false);
  used[0] = true;
  for (let i = 1; i < pts.length; i++) {
    const last = result[result.length - 1];
    let bestIdx = -1, bestDist = Infinity;
    for (let j = 0; j < pts.length; j++) {
      if (used[j]) continue;
      const dx = pts[j].x - last.x, dy = pts[j].y - last.y, dz = pts[j].z - last.z;
      const d = dx*dx + dy*dy + dz*dz;
      if (d < bestDist) { bestDist = d; bestIdx = j; }
    }
    if (bestIdx === -1) break;
    used[bestIdx] = true;
    result.push(pts[bestIdx]);
  }
  return result;
}
```

**结果**：问题未解决，说明不是点顺序问题。

---

### 第2步：打印后端返回数据结构

在后端 `slicer` 接口加 log，查看 `HGP_Slicer_Mesh` 返回值的结构：

```python
offsetses, offsets = hgp_py.HGP_Slicer_Mesh(
    mesh_path, req.plane_normal, req.plane_ds
)
print("offsetses 长度:", len(offsetses))
print("offsets 长度:",   len(offsets))
for i, c in enumerate(offsets):
    print(f"  offsets[{i}] 长度: {len(c)}")
```

**输出**：
```
offsetses 长度: 5
offsets   长度: 10
  offsets[0] 长度: 6
  offsets[1] 长度: 10
  offsets[2] 长度: 7
  ...
```

**发现**：`offsetses` 长度为 5（等于切平面数），`offsets` 长度为 10（是切平面数的 2 倍）。原始代码只用了 `offsets`，**把每个切平面的 2 段轮廓当成了 10 个独立切平面**，导致轮廓不完整。

---

### 第3步：确认 offsetses 的二维结构

继续打印 `offsetses[0]` 的内容：

```python
print("offsetses[0] 长度:", len(offsetses[0]))
print("offsetses[0][0] 点数:", len(offsetses[0][0]))
print("offsetses[0][1] 点数:", len(offsetses[0][1]))
```

**输出**：
```
offsetses[0] 长度: 2
offsetses[0][0] 点数: 6
offsetses[0][1] 点数: 10
```

**结论**：`offsetses[i]` 是第 `i` 个切平面的所有轮廓段列表，每个切平面包含 2 段；`offsets` 是所有段的扁平化列表，结构层级丢失，不能直接使用。

---

### 第4步：分析为什么每个切平面有 2 段

打印每段的实际坐标，观察到：

- 每段点的某一轴坐标完全相同（即同一切平面上）
- 2 段的首尾坐标非常接近，例如：

```
段0 尾: [1.554, y, 0.608]
段1 尾: [1.553, y, 0.594]  ← 距离极近
```

**原因**：CGAL 切片时对凸包的切面会产生两段不相连的轮廓（切到了凸包的两侧区域），这两段实际上应该首尾拼接成一条完整的封闭轮廓。

---

### 第5步：确认 OBJ 文件本身无问题

由于测试网格是随机凸包，曾怀疑 OBJ 文件面片法向不一致。验证：

```python
print("OFF路径:", mesh_path)
print("文件存在:", os.path.exists(mesh_path))
```

输出 `文件存在: True`，且 `save_mesh` 已设置 `repair=True`，文件格式无误，排除文件问题。

---

## 三、根本原因

`HGP_Slicer_Mesh` 返回两个变量：

| 变量 | 含义 | 结构 |
|---|---|---|
| `offsetses` | 按切平面分组的轮廓段 | `list[list[list[point]]]` |
| `offsets` | 所有轮廓段的扁平列表 | `list[list[point]]` |

原始代码错误地用了 `offsets`，导致：
1. 每个切平面的 2 段被当成 2 个独立切平面渲染
2. 每段只有部分轮廓点，无法构成完整截面
3. 同一切平面的两段没有拼接，轮廓不闭合

---

## 四、修复方案

### 后端：拼接同一切平面的多段轮廓

新增 `join_contour_segments` 函数，把同一切平面的多段按首尾距离贪心拼接成一条完整轮廓：

```python
def join_contour_segments(segments, tol=1e-3):
    """把同一切平面的多段轮廓首尾拼接成一条完整轮廓"""
    if not segments:
        return []
    if len(segments) == 1:
        return segments[0]

    remaining = [list(seg) for seg in segments]
    result = remaining.pop(0)

    while remaining:
        tail = result[-1]
        best_idx, best_rev, best_dist = -1, False, float('inf')

        for i, seg in enumerate(remaining):
            d0 = sum((tail[k] - seg[0][k])**2 for k in range(3))
            d1 = sum((tail[k] - seg[-1][k])**2 for k in range(3))
            if d0 < best_dist:
                best_dist, best_idx, best_rev = d0, i, False
            if d1 < best_dist:
                best_dist, best_idx, best_rev = d1, i, True

        seg = remaining.pop(best_idx)
        if best_rev:
            seg = seg[::-1]
        result.extend(seg)

    return result
```

修改 `/api/slicer` 接口，改用 `offsetses` 并调用拼接函数：

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
                "contours": [join_contour_segments(plane_contours)]
            }
            for plane_contours in offsetses
        ]
    }
```

**注意**：返回字段从 `contour`（单段）改为 `contours`（数组），为后续支持多段输出保留扩展性。

---

### 前端：适配新的 contours 字段

```javascript
// 改前（使用 contour 单字段）
data.slices.forEach((s, idx) => {
  const rawPts = s.contour.map(p => new THREE.Vector3(p[0], p[1], p[2]));
  if (!rawPts.length) return;
  pts.push(pts[0]);
  scene.add(new THREE.Line(...));
});

// 改后（使用 contours 数组，同一切平面同色）
data.slices.forEach((s, idx) => {
  const color = colors[idx % colors.length];
  s.contours.forEach(contour => {
    const pts = contour.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    if (!pts.length) return;
    pts.push(pts[0]); // 闭合
    scene.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color })
    ));
  });
});
```

---

## 五、经验总结

| 排查步骤 | 结论 |
|---|---|
| 前端加排序函数 | 无效，点本身已有序，问题不在排序 |
| 打印后端返回结构 | 发现用了扁平化的 `offsets` 而不是分组的 `offsetses` |
| 打印 offsetses 结构 | 确认每个切平面有 2 段，需要拼接 |
| 分析坐标数据 | 确认 2 段首尾相邻，可以贪心拼接 |
| 排查 OBJ 文件 | 文件无问题，repair=True 已设置 |

**核心教训**：`HGP_Slicer_Mesh` 返回两个变量，`offsetses` 是按切平面分组的正确结构，`offsets` 是扁平化的副产物。应始终使用 `offsetses`。