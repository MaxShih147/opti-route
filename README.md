# Bus Route + Stops Optimizer

題目二：公車路線與站點設置規劃的互動式 demo。

> A web app + Python backend that jointly optimizes a bus route from A to B
> and the placement of up to K intermediate stops, balancing route operating
> cost, stop-building cost, and passenger walking cost.

## 快速啟動

```bash
/opt/homebrew/bin/python3.12 -m venv .venv
.venv/bin/pip install -r backend/requirements.txt
.venv/bin/uvicorn backend.main:app --reload --port 8765
# open http://localhost:8765/
```

## 專案結構

```
backend/
  graph_gen.py          隨機城市無向圖產生器（擾化網格 + 主幹道 + 地形 + 禁區）
  models.py             Pydantic / dataclass 共用型別
  main.py               FastAPI；/api/{scene,generate,solve,edit}
  algorithms/
    common.py           Dijkstra walk-distance, 共用 p-median 解
    two_phase.py        演算法 B  — Dijkstra + 路上 p-median
    ksp.py              演算法 C  — Yen K-shortest paths + corridor p-median
    mip.py              演算法 A  — CP-SAT 聯合 MIP（含 MTZ 防 subtour）
frontend/
  index.html, styles.css, app.js      單頁 SVG 互動視覺化
docs/                                  原題目
```

---

## 1 · 問題建模

設城市為一張**無向加權圖** `G = (V, E)`：

- 節點 `V`：路口 / 候選站點 / 乘客 snap 位置 / 起點 A、終點 B
- 邊 `E`：道路；每條邊權重 `w_e = length × terrain`（公車營運成本代理）
- 禁區：對公車而言被移除，但行人可繞行
- 乘客 `P = {p_1, ..., p_n}`：每位 snap 到最近節點

**決策變數**

| 變數 | 範圍 | 意義 |
|---|---|---|
| `x_{u→v}` | {0,1} | 公車是否走有向弧 u→v |
| `s_v` | {0,1} | 節點 v 設站（A、B 強制為 1） |
| `z_{p,v}` | {0,1} | 乘客 p 指派至站 v |

**目標（最小化）**

```
min   α · Σ_{(u,v) ∈ E_bus} w_uv · x_{u→v}      (路線營運)
    + γ · Σ_{v ∉ {A,B}} (c_fix · terrain_v) · s_v  (站建設)
    + β · Σ_{p, v} walk(p, v) · z_{p,v}          (乘客步行)
```

`α, β, γ` 即題目所要的「不同成本權重」。

**約束**

- 流量守恆：A 是 1 單位流的來源，B 是匯，其餘節點守恆
- `in(v) ≤ 1` 對非端點：路線是簡單路徑
- MTZ 勢函數 `u_v`：嚴格遞增 → 排除 subtour
- `s_v ≤ in(v)`：站點必須在路線上
- `Σ_{v ∉ {A,B}} s_v ≤ K`：最多 K 個中繼站
- `Σ_v z_{p,v} = 1`，`z_{p,v} ≤ s_v`：乘客指派

**模型本質：**「constrained facility location + path planning」混合問題，是 prize-collecting Steiner tree 與 p-median 的綜合。NP-hard。

---

## 2 · 三套求解策略（並陳比較）

### B · 兩階段分解  `two_phase.py`

1. **Phase 1**：在公車子圖（禁區邊已剔除）上 Dijkstra 找 A→B 最短路徑 π。
2. **Phase 2**：以 π 上的節點為候選站，計算每位乘客到每個候選站的步行距離（全圖最短路徑），用 **greedy add → 1-swap local search → 1-drop** 解 p-median。

| 優 | 缺 |
|---|---|
| 最快（~10 ms 級）、直觀、可解釋 | 路線「先決定」、不考慮乘客分布；乘客集中於非主幹線時步行成本爆炸 |

適用：UI 即時拖拉、即時回饋；或當作其他演算法的初始解。

### C · K-最短路徑 + Corridor p-median  `ksp.py`  ⭐ 主推

1. **Yen's K-shortest simple paths** 取 K 條（預設 6）候選 A→B 路線。
2. 對每條 π：取「π ∪ π 周圍 r-hop 內節點」為候選站集合 — 即**走廊（corridor）**，這讓選址可微幅偏離主軸往乘客密集區靠。
3. 對每條 π 解 p-median；若選到 corridor 內非 π 上的站，做**路線修補**：依 stop 沿主軸投影排序後 `Dijkstra(A → s_1 → s_2 → … → s_k → B)`，這允許 spur 支線。
4. 比較所有候選的 `α·route + Σstops + β·walk`，取最低者。

| 優 | 缺 |
|---|---|
| 路線「被乘客吸引」會彎曲；中型場景平衡好；幾乎都比 B 顯著低 | 仍是 heuristic；K、corridor 半徑要調 |

適用：實務工程系統的主力 — 規劃時間預算數百 ms 到秒級。

### A · MIP (CP-SAT)  `mip.py`

完整 MILP 用 Google OR-Tools CP-SAT 求解：

- 雙向弧 + 流量守恆 + 簡單路徑（每節點 inflow ≤ 1）
- **MTZ 勢函數**強制 `u_b ≥ u_a + 1 − N·(1−x_{a→b})`：排除任何（連同 disconnected）subtour
- 每位乘客的 z 候選只取「步行最近 M=40 個」減少變數規模

| 優 | 缺 |
|---|---|
| **最佳性保證**（含 gap 報告）；可當 B/C 的金標 | 規模 > ~50 乘客或 > ~200 節點開始吃力；20s 內可能只到 FEASIBLE |

適用：驗證、小規模規劃、合約報告需要的最佳性證明。

---

## 3 · 也考慮過、但這次沒實作的方案

| 方法 | 為何不選 |
|---|---|
| **Iterative EM**：交替「固定站算路徑 / 固定路徑算站」 | 與 KSP 屬同類局部搜尋；KSP 更顯式列舉路線、解釋性強 |
| **Genetic Algorithm / SA** | 對這題的結構利用低；參數調整成本高；若進入超大規模 (>500 nodes / >200 passengers) 才開始有優勢 |
| **Steiner tree on prize collecting** | 學術正確但實作門檻高；本題乘客分布的「轉折優勢」用 KSP corridor 已能掌握大半 |
| **RL** | 訓練/收斂成本不划算；只適合長期動態調度 |

---

## 4 · 演算法之間的洞察

從 web 上反覆觀察 (建議自己拉)：

1. **乘客集中時 KSP 大勝 two-phase**：當乘客明顯位於某個側翼，KSP 會選擇一條較長但偏向那側的路徑，雖 route 成本上升但 walk 大降，總成本仍贏 30~50%。
2. **β 越大 → KSP 越敢繞路**；β = 0 退化成「兩階段就好」。
3. **MIP 在小場景 (≤ ~50 節點 / ~20 乘客) 通常 < 1 秒 OPTIMAL**；大場景 15 秒 FEASIBLE，gap 可能 > 50%（這正是 demo 想呈現的：「為什麼工程上要用 KSP」）。
4. **MIP 強制簡單路徑（MTZ）**，KSP 允許 spur 支線。當「繞去服務一個遠處乘客再原路回來」最划算時，KSP 會找到 MIP 找不到的解。這是兩種模型語意的差異而非 bug。

---

## 5 · 模型/實作的限制 (Limitations)

- **離散圖近似**：城市抽象成圖；現實的「站可以設在道路任意位置」未建模。需要更高解析度可細化道路為多個 sub-segment。
- **步行距離用 full-graph 最短路**：實務應分開「步行網」與「道路網」（如 OSM 雙網層），這裡假設兩者相同。
- **無容量、無班次、無時刻表**：純空間靜態問題。若要加 vehicle routing 動態元素，需擴充為 VRP / 時刻表合成。
- **MIP 簡單路徑限制**：對應「公車不重複路段」假設；現實的環狀路、spur 服務需改用 capacitated multi-arc flow 重新建模。
- **單目標純成本**：題目要的「不同權重」用 α/β/γ 線性組合處理；若要 Pareto-front 視覺化，需多目標求解（ε-constraint / NSGA-II）。

---

## 6 · 使用 demo

打開 [http://localhost:8765/](http://localhost:8765/) 後：

1. 右側上方調 `rows/cols/passengers/seed`，按 **重新生成城市**。
2. **編輯模式** 可以：移動 A、移動 B、增刪乘客。
3. 拉 **K / α / β / 站成本** 三條滑桿。
4. 連按 **B → C → A** 三個演算法，觀察結果疊在地圖上 + 右下角比較表。
5. 比較表中**綠色那列**是當前最佳。

希望這份 demo 能直接支撐口頭討論時的「主觀判斷與評估洞察」。
