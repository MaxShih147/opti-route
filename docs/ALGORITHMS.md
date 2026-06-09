# opti-route · 演算法詳述

公車主路線 + 站點選址的聯合最佳化問題、兩種解法、設計權衡。本文寫得夠完整可以直接被當成簡報草稿。

---

## 1. 問題定義

### 1.1 場景

城市裡有 N 個路口、若干段道路；少數區域是公車不可穿越的禁區（窄巷、行政限制等）。乘客零星分布在城市各處（往往會聚集成 hotspots）。

設計者要規劃一條公車路線：

- 從起點 **A**（總站）出發到終點 **B**（目的地）
- 路上設最多 **K** 個中繼站
- 乘客**步行**到最近的站搭車
- A、B 本身永遠是站（既有設施、零建設成本）

### 1.2 目標

最小化三項加權成本：

$$
\min \; \alpha \cdot C_{\text{route}} \;+\; \gamma \cdot C_{\text{stops}} \;+\; \beta \cdot C_{\text{walk}}
$$

| 項目 | 內容 |
|---|---|
| `C_route` | 公車所走邊的權重總和（長度 × 地形係數） |
| `C_stops` | Σ 每個中繼站的固定建設成本 × 該節點地形係數 |
| `C_walk` | Σ 乘客數 × 步行距離（demand-weighted） |

α / β / γ 是設計者可調的權重。γ 內嵌進 `stop_fixed_cost` 沒另外開 slider — 跟整體成本同量級即可。

### 1.3 限制

- 路線避開禁區邊（嚴格不可走）
- 行人可穿越禁區（兩張圖）— 公車用 `bus_subgraph`、乘客用 full `G`
- 站只能設在節點上
- 每位乘客必須指派到唯一一個站

### 1.4 本質

「constrained facility location + path planning」的耦合最佳化問題，
NP-hard。是 **Prize-Collecting Steiner Tree** 跟 **p-median** 的綜合，
中間還夾著 graph constraint（禁區、地形）。

---

## 2. 決策變數與 MIP 建模

### 2.1 變數

| 變數 | 範圍 | 意義 |
|---|---|---|
| `x_{u→v}` | {0,1} | 公車有沒有走有向弧 u→v（每條無向邊拆成兩個方向弧） |
| `s_v` | {0,1} | 節點 v 設站（A、B 強制為 1） |
| `z_{p,v}` | {0,1} | 乘客 p 指派至站 v |
| `u_v` | ℤ in [0, N] | MTZ 勢函數（v=A 時鎖在 0） |

### 2.2 目標函數

$$
\min \sum_{(u,v)} \alpha\,w_{uv}\, x_{u \to v}
\;+\; \sum_{v \notin \{A,B\}} c_{\text{fix}}\, \text{terrain}_v\, s_v
\;+\; \sum_{p,v} \beta\, \text{demand}_p\, \text{walk}(p,v)\, z_{p,v}
$$

`walk(p, v)` 預先用 Dijkstra 算出來、是常數，所以目標函數仍然線性。

### 2.3 約束

**流量守恆**（路徑形狀）：

$$
\sum_j x_{Aj} = 1,\quad \sum_j x_{jA} = 0\\
\sum_j x_{jB} = 1,\quad \sum_j x_{Bj} = 0\\
\sum_j x_{jv} - \sum_j x_{vj} = 0 \quad \forall v \notin \{A,B\}\\
\sum_j x_{jv} \le 1 \quad \forall v \notin \{A,B\}
$$

A 是 1 單位的源、B 是匯、其他節點守恆。`in(v) ≤ 1` 強制簡單路徑。

**MTZ subtour elimination**（防止 disconnected cycles）：

$$
u_b \;\ge\; u_a + 1 - N \cdot (1 - x_{a \to b}) \quad \forall (a,b),\; b \ne A
$$

直白讀：如果走過 a→b，則 b 的勢必須嚴格大於 a 的勢。
排除任何 cycle（含跟主路徑斷開的子環）。

> **早期 debug 經驗**：沒有 MTZ 時，CP-SAT 會「**偷塞一個沒接到主路徑的子環**」、藉此讓 `s_v=1` 在子環節點上、減少 walking。視覺上會看到「站牌沒在路線上」的詭異結果。

**站點在路徑上**：

$$
s_v \;\le\; \sum_j x_{jv} \quad \forall v \notin \{A,B\}
$$

不在路徑上的節點 inflow=0 → s_v 被夾死成 0。

**站點預算**：

$$
\sum_{v \notin \{A,B\}} s_v \;\le\; K
$$

**乘客指派**：

$$
\sum_v z_{p,v} = 1 \quad \forall p,\quad z_{p,v} \le s_v
$$

每位乘客指派到唯一一站、且該站必須是 active。

### 2.4 變數規模

對 200 node、350 bus edge、100 乘客、M=20 候選站 per passenger：

- x vars: 700（2 × 350 arcs）
- s vars: 200
- z vars: 2000
- u vars: 200
- **Total ~3100 binary + 200 integer**

CP-SAT 在合理時間內能解，但需要不少 tricks（見 §4）。

---

## 3. KSP — K-最短路徑啟發式

### 3.1 流水線

```
1. Yen's algorithm → K 條 A→B 候選路徑（依 cost 排序）
2. For each path π:
     a. corridor = π ∪ π 周圍 r-hop 內節點
     b. 在 corridor 上跑 p-median (greedy + 1-swap + 1-drop)
     c. 若有 stop 在 corridor 內但不在 π 上
        → 路線修補：Dijkstra(A → s₁ → s₂ → ... → s_k → B)
3. 比較所有候選的 α·route + Σstops + β·walk
4. 回傳最低總成本者
```

### 3.2 Yen's K-Shortest Paths 直觀

```
P₁ ← Dijkstra(A→B)
For k=2 to K:
    For each spur node v on P_{k-1}:
        ignore the edge (v → next-on-P_{k-1}) and known used edges
        run Dijkstra(v → B) on the modified graph
        candidate = root(P_{k-1}, v) + spur
    Pick lowest-cost candidate not yet seen.
```

複雜度 O(K · (V log V + E))，networkx 有內建：`nx.shortest_simple_paths()`。

### 3.3 Corridor expansion

```
corridor_nodes := {π 上所有節點}
repeat r times:
    corridor_nodes ∪= (corridor_nodes 的鄰居)
```

跑在 **full graph G**（含禁區邊）— 允許 corridor 涵蓋 spur 可達區域。
重要：之後得篩掉那些不在 `bus_component` 的點，
否則路線修補的 Dijkstra 會死在 NetworkXNoPath（看 §6.3 bug 故事）。

### 3.4 p-median (UFLP-with-budget)

```
chosen := {}
# greedy add
while len(chosen) < K and improvement > 0:
    pick the candidate that most reduces β·Σdemand·walk + Σstops
# 1-swap
for s_out in chosen, s_in in candidates:
    swap if it lowers total cost
# 1-drop (over-added safety)
for s_out in chosen:
    drop if it lowers total cost
```

時間複雜度 O(|cand|² · K · |pass|)，對 ~50 candidates × 100 passengers 是亞秒級。

### 3.5 路線修補（spur 來源）

```
chosen 用沿 π 方向投影排序 → ordered = [s₁, s₂, ...]
waypoints = [A] + ordered + [B]
final_path = []
for (a, b) in pairs(waypoints):
    seg = Dijkstra(a → b, bus_subgraph)
    final_path += seg
```

**spur 就是這裡誕生的**：如果 s_i 偏離主軸，`Dijkstra(s_{i-1} → s_i)` 會繞過去，然後 `Dijkstra(s_i → s_{i+1})` 可能繞回主軸，造成節點被重複經過。

> MIP 因為 MTZ 強制簡單路徑、表達不了 spur；KSP 允許。
> 這是兩種方法**最深層的可行解空間差異**，也是大場景 KSP 反勝 MIP 的原因（見 §5）。

### 3.6 Auto-tune (基於圖規模)

empirical sweep 顯示：

- `k_paths` plateaus past 6（k=3 跟 k=20 在大 corridor 時結果相同）
- `corridor_hops` 跟 √N 成正比

實作：

```python
k_paths = 6
corridor_hops = clamp(3..6, round(0.37 · sqrt(N)))
```

| 規模 N | corridor_hops |
|---|---|
| 80 | 3 |
| 130 | 4 |
| 200 | 5 |
| 320 | 6 |

---

## 4. MILP — CP-SAT 完整模型

### 4.1 求解器選擇

用 Google OR-Tools 的 **CP-SAT**：
- 對 boolean-heavy MIP 是 state-of-the-art
- 內建 LP relaxation + branch-and-bound + cuts
- 支援 hint（warm-start）
- 不需要授權

### 4.2 解出來怎麼用

```python
status = solver.Solve(model)
if status in (OPTIMAL, FEASIBLE):
    # x_{u→v} = solver.Value(x_var)
    # 重建路徑：從 A 往下追 used_arcs 直到 B
    # 從 solver.Value(s[v]) 收集 chosen stops
    # 從 solver.Value(z[(p,v)]) 收集 assignments
```

CP-SAT 對成本要求 INTEGER，所以我們把所有成本 × `SCALE = 100` 後 round 成 int。

### 4.3 Phase 1 加速（v1.0）

幾個堆疊招數：

| 招 | 原理 | 收益 |
|---|---|---|
| **Warm-start hints from KSP** | `model.AddHint(x_arc, 1 if in KSP path else 0)` etc.；CP-SAT 從近似最佳解起跳 | UB 立即可用、剪掉大量無望分支 |
| **M = 40 → 20** | 每位乘客只考慮 M-最近的站當候選 → z 變數量減半 | 變數 −50% |
| **`linearization_level = 2`** | 要 CP-SAT 加更多 LP cuts | LB 更緊 |
| **`num_search_workers = 16`** | 平行多策略搜索（Mac Studio 28 核） | 1.5-2× speedup |

Phase 1 結果：
- 80n 場景：OPTIMAL 2.5s
- 200n 場景：OPTIMAL 6-17s（之前 timeout）

### 4.4 Phase 2 加速（v2.0）

**KSP corridor restriction**：MIP 不在整張圖上跑，
而是在「Yen 路徑家族 ± BFS hops」這個子圖內跑。

```python
mip_hops = clamp(4..7, round(0.37 · sqrt(N)))
corridor_nodes := union(Yen paths) ∪ N-hop BFS expansion
Hbus := Gbus.subgraph(corridor_nodes)
# x, s, z, u 全部只對 corridor 內節點建變數
```

Floor=4 因為 MIP 的 corridor **移除節點**（KSP corridor 只限制站點候選），
所以 MIP 需要稍寬的安全邊際。

Phase 2 結果：
| 規模 | hops | runtime |
|---|---|---|
| 80n | 4 | **0.3s OPTIMAL** |
| 132n | 4 | **0.4s OPTIMAL** |
| 208n | 5 | **1.0s OPTIMAL** |
| 320n | 7 | **8.9s OPTIMAL** |

從 v1 的「default 場景 90s timeout、gap 70%」變成「**1 秒內 OPTIMAL**」，~90× speedup。

---

## 5. Benchmark：兩種方法並排比較

### 5.1 同場景對比（seed=42, 100 passengers）

```
規模    | KSP cost  ms   | MIP cost  ms     | KSP gap
80n     |   8181  110   |   7637   278     | +7.1%
132n    |  10905  210   |  10953   470     | -0.4%   ← KSP 領先（spur）
208n    |  25604  325   |  26147  1635     | -2.1%   ← KSP 領先（spur）
320n    |  57721  480   |  53455  8893     | +8.0%   ← MIP 領先（簡單路徑足夠）
396n    |  64368  810   |  75764  30s (timeout) | -15%  ← KSP 大勝
```

### 5.2 觀察

**為什麼 KSP 在某些場景反勝 MIP？**

- KSP 的可行解集合 ⊋ MIP 的可行解集合（KSP 含 spur）
- MIP 在 200+ 節點 + 5 站 budget 時，simple-path 限制變成主要 bottleneck
- spur 讓「繞去服務一個遠處乘客再回來」這種策略成為可能

**為什麼小場景 MIP 領先？**

- 圖太小、節點少 → 即使允許 spur，最佳解也通常是 simple path
- MIP 的「全空間搜索」優勢顯現

**這個 trade-off 是「演算法 vs 模型」的差異，不是其中一個比較強。**

### 5.3 規模 vs 時間（log scale）

```
Time (s) │
   30s   │                                            ▆ MIP
    8s   │                                  ▆
    1s   │                        ▆
  100ms  │              ◆       ▆
   50ms  │     ◆      ◆       ◆ KSP
   10ms  │   ◆
         └─────────────────────────────────────────────  Nodes
            50    80   130   200   320   400
```

KSP 線性增長、MIP 接近指數。實務界線約在 300 nodes 上下。

---

## 6. 重要實作 lessons

### 6.1 模型解讀的隱含假設（題目歧義）

下列假設我做了取捨；若要套到別處需重新評估。

| 議題 | 我的解讀 | 替代 |
|---|---|---|
| 「站建設成本 + 地形加成」中的「+」 | 乘法 `fixed × terrain` | 加法 `fixed + offset` |
| 「公車路線可轉折」是否允許 spur？ | KSP：可、MIP：不可 — 兩個並陳 | 只有單一模型 |
| 乘客「步行」用什麼圖 | full graph（行人可穿禁區） | bus_subgraph |
| 站點位置 | 只能在節點上 | 邊上任一點（連續） |
| A、B 是否收建設費 | 不收（既有設施） | 都收 |
| 乘客有 demand 嗎 | 1-6 人，per point | 都是 1 |

### 6.2 graph_gen 的禁區生成

不是簡單圓盤，是**阿米巴 polygon**：
1. 取圓心 + base radius
2. 3 個正弦諧波 (k=2, 3, 5) 擾動 radius
3. 28 點環狀採樣 → 平滑 polygon

點是否在禁區：ray-cast point-in-polygon。

**兩個 hard caps 確保場景合理**：
- 每禁區覆蓋路口 ≤ 4（規模 < 200）或 ≤ 8（否則）
- 任兩禁區共享路口 = 0

實作邏輯：嘗試最大半徑 → 若違規縮小 → 40 次 attempt 還不行就跳過該禁區。

### 6.3 KSP「孤兒站」debug 故事

**症狀**：小場景下，路線跑出來，但**站牌不在路線上**。

**根本原因**：
- `corridor expansion` 用 `G.neighbors()` 含禁區邊
- 所以 corridor 可能涵蓋「在 G 上靠近 path 但在 `bus_subgraph` 上跟 A 不連通」的節點
- p-median 把這種節點選為站
- 路線修補的 Dijkstra(A→s) 在 Gbus 找不到路 → fallback 用原 geodesic
- 但 chosen_stops 沒清掉 → 路線跟站不符

**修法**：
- 預先算 `bus_component = node_connected_component(Gbus, A)`
- 候選站必須屬於這個 component
- 修補失敗則 skip 該 path candidate

詳見 commit `f01e8b7`。

### 6.4 MIP 的「子環走私」debug 故事

**症狀**：早期版本（沒 MTZ）下，MIP 找到「總成本很低」的解，但
站牌跟主路線之間有**沒接到的子環**（cycle 旁路）。

**根本原因**：
- 沒 MTZ 時，subtour（沒接到主路徑的環）是 MIP-可行的
- subtour 上的節點 `s_v` 可以為 1（合法被選為站）
- 因為「站可以分流附近乘客」→ MIP 用 subtour「偷塞站」
- 但這個解物理上不合理：公車沒有實際走到那

**修法**：加入 MTZ 勢函數約束
```
u_b ≥ u_a + 1 - N · (1 - x_{a→b})
```
強制節點順序遞增 → 任何 cycle 都會違反。

### 6.5 KSP 反勝 MIP — 這是 feature 不是 bug

每次 MIP cost 高過 KSP，新手會問「不是 MIP 是最佳化嗎？」

答：MIP 是「**模型 P1（簡單路徑）內**」的最佳。KSP 跑的是 P2（允許 spur）。
P2 ⊋ P1，所以 P2 的 optimum ≤ P1 的 optimum。

要讓 MIP 真的勝出，需要把模型改成「允許 spur 的 capacitated multi-arc flow」 —
變數量會爆炸，runtime 上限會大幅下降。不值。

---

## 7. 未來方向

| 想法 | 預期收益 | 工作量 |
|---|---|---|
| HiGHS-WASM 純前端版本 | 零後端、無 cold start、永久免費 | 已有 branch `static-no-backend`、待精進 |
| Lagrangian relaxation | LB 更緊、可解大場景 | 中等 |
| Column generation (路徑為決策變數) | 處理超大圖 | 大 |
| Multi-objective / Pareto front | 給設計者一條 trade-off 曲線 | 中等 |
| Dynamic / Time-dependent | 加上班尖峰、班次規劃 → 真實公車排程 | 大 |

---

## 附錄 A：原始 benchmark 資料

`docs/bench_ksp_data.json` — 44 種設定（k_paths × corridor_hops × seeds × 場景大小）的 KSP gap 對 MIP 結果。可用於：
- 重現本文中的 trade-off 圖
- 訓練自動參數調整
- 做學術 figure

## 附錄 B：相關文獻

- Yen, J.Y. (1971). "Finding the K Shortest Loopless Paths in a Network"
- Miller, Tucker & Zemlin (1960). "Integer Programming Formulation of Traveling Salesman Problems"
- Daskin, M.S. (2008). "Network and Discrete Location" — facility location 經典
- Laporte, G. (2009). "Fifty Years of Vehicle Routing"
- HiGHS solver: https://highs.dev
- Google OR-Tools CP-SAT: https://developers.google.com/optimization/cp/cp_solver
