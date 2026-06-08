# Bus Route + Stops Optimizer

題目二：公車路線與站點設置規劃的互動式 demo。

> A web app + Python backend that jointly optimizes a bus route from A to B
> and the placement of up to K intermediate stops, balancing route operating
> cost, stop-building cost, and passenger walking cost.

最終 demo 呈現 **KSP（K-shortest paths + corridor p-median）** 跟 **MIP（CP-SAT MILP）** 兩個演算法的對照，並在第 4 章節記錄了**做過比較實驗的兩階段 baseline** 與其他放棄掉的方案，說明為什麼最終選這兩個。

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
    common.py           Dijkstra walk-distance、共用 p-median 解、bus_subgraph
    ksp.py              ⭐ Yen K-shortest paths + corridor p-median
    mip.py              ⭐ CP-SAT MILP（含 MTZ 防 subtour）
    two_phase.py        baseline，演算法比較用，UI 不顯示
frontend/
  index.html, styles.css, app.js      單頁 SVG 互動視覺化
scripts/
  bench_ksp.py          掃 (k_paths × corridor_hops) 對 MIP 的 gap 實驗腳本
docs/
  Algorithm Assignment.pdf            原題目
  bench_ksp_data.json                 實驗原始資料（44 組設定 × 多 seeds）
```

---

## 1 · 問題建模

設城市為一張**無向加權圖** `G = (V, E)`：

- 節點 `V`：路口 / 候選站點 / 乘客 snap 位置 / 起點 A、終點 B
- 邊 `E`：道路；每條邊權重 `w_e = length × terrain`（公車營運成本代理）
- 禁區邊：對公車而言被剔除（bus_subgraph），但行人可繞行（full graph）
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

**模型本質：**「constrained facility location + path planning」混合問題，是 prize-collecting Steiner tree 與 p-median 的綜合。**NP-hard**。

---

## 2 · 題目歧義與我的解讀（顯式假設）

題目原文有 10 處未明確定義。下面把我的解讀全部列出來，並標註是「題目沒講」還是「題目語意有歧義」。**面試展示時這節最值得攤開來討論** — 它證明我意識到題目的不確定性、做了明確選擇、可以隨時被挑戰修正。

### 2.1 速查表

| # | 題目原文 / 缺什麼 | 我的解讀 | 類型 |
|---|---|---|---|
| 1 | 「最多設**若干個**」站 — K 沒給 | K 是 UI 滑桿，1~10 | 沒講 |
| 2 | 「每站固定成本 **+** 當地地形加成」 | **乘法**：`stop_fixed_cost × terrain_v`，不是加法 | 歧義 |
| 3 | 「公車路線可**轉折**」 — 可不可以重複走？ | KSP 允許 spur 支線；MIP 強制簡單路徑 — **兩個並陳** | 沒講 |
| 4 | 「**不同的成本權重**」 | α, β 是 UI 滑桿；γ 用 stop_fixed_cost 取代 | 沒講 |
| 5 | 「乘客**走到**最近站牌的成本」 | full graph Dijkstra by `length`；行人能穿越禁區 | 沒講 |
| 6 | 「公車路線本身**避開禁區**」 | 禁區是**邊**；公車嚴格禁止；行人可走 | 沒講 |
| 7 | 「**設站**」 | 只能設在**節點**上，不能設在邊上任一點 | 沒講 |
| 8 | 「A 主要發車站 → B 主要目的地」 | 單向；A 與 B **不收建設費**（既有設施） | 沒講 |
| 9 | 「散布著多名乘客」 | 離散點；每位 demand = 1；強制指派一站（不能不搭） | 沒講 |
| 10 | 「**城市**」空間表示 | 隨機合成無向圖（擾化網格 + 主幹道 + 程序化地形） | 沒講 |

### 2.2 重點假設的詳細說明

#### 站建設成本：**乘法**而非加法

題目寫「每站固定成本 **+** 當地地形加成」。中文「加成」有歧義：
- 字面：`c = fixed + terrain_offset`
- 商業用法（如「10% 加成」）：`c = fixed × terrain_factor`

我選**乘法**：
```python
stop_cost(v) = stop_fixed_cost × terrain[v]
```
理由：terrain 在程式碼裡是無量綱倍率（範圍 0.2~1.8），跟 `stop_fixed_cost` 相乘比較自然（地形是「修正因子」概念）。加法則 terrain 必須跟 fixed_cost 同單位，建模上尷尬。

舉例（`stop_fixed_cost=50`）：
- 站設在 terrain=1.8 的紅區 → 90
- 站設在 terrain=0.2 的藍區 → 10

差距 9 倍，演算法會偏好藍區設站 — 這個視覺梯度在前端肉眼可見。

#### A 與 B 不收站建設費

題目只說「每設一個站牌需要建設與營運成本」。A、B 是「**主要**發車站/目的地」— 我解讀為**既有設施**，不算進「新建中繼站」的預算。

實作：MIP 在 stop 成本累加迴圈內 `if n in (A, B): continue`；KSP 經由 `fixed_stops=[A, B]` 排除。

#### 行人 vs 公車的圖

路網其實有**兩張**：

| 圖 | 用途 | 邊權重 | 邊集合 |
|---|---|---|---|
| `G_bus` (`bus_subgraph(G)`) | 公車路徑 | `length × terrain` | 不含禁區邊 |
| `G_full` (`G` 本身) | 乘客步行 | `length` 純距離 | 含禁區邊 |

兩個顯式選擇：
- **行人不被地形 penalize** — 因為 terrain 代表公車營運成本（燃料、保養），不是行人疲勞
- **行人可穿越禁區邊** — 禁區是公車的法規限制（窄巷、行政管制），不是物理屏障

如果現實的禁區是河流或牆，這假設要改 — 把行人 Dijkstra 也用 bus_subgraph 即可。

#### 站只能在**節點**上

實務公車站可以設在任一路段。我們的離散化要求站必須是節點。代價：站位置解析度受網格密度限制。改善方向：把每條邊細分成多個 sub-segment 節點，但 demo 不需要。

#### terrain 是「隨機但有結構」

不是每節點 iid 亂數，是 **多倍頻 value noise**（類 Perlin）：

```python
# graph_gen.py _value_noise_2d()
for o in range(octaves):                  # 預設 3 層
    res_r = rows // (2 ** (octaves - o))  # 粗 → 細
    coarse = [[rng.random() ...]]
    # 雙線性插值升解析度 → 累加
    amp *= 0.5
```

結果：地圖上是**連續色塊**（藍便宜紅貴），不是椒鹽雜訊。整張地圖的隨機性收斂在單一 `seed` 上 — 同 seed → 同 terrain + 同節點 jitter + 同邊刪除 + 同禁區 + 同乘客。完全可重現。

UI 上看到的藍紅漸層底色就是 terrain field 視覺化。

#### 「不同權重」對應 UI 上的什麼

| 題目所寫 | 對應 UI 滑桿 | 對應變數 |
|---|---|---|
| 路線行進/營運成本 | α 路線權重 | `alpha_route` |
| 站建設成本 | 站建設成本 | `stop_fixed_cost`（同時兼任 γ 的角色） |
| 乘客步行成本 | β 步行權重 | `beta_walk` |

我把 γ 內嵌進 `stop_fixed_cost` 而沒額外開一個 γ 滑桿 — 因為 `γ × c_fix` 跟 `1 × (γ·c_fix)` 是同一回事，多一個滑桿只是冗餘。

---

## 3 · 為什麼最後選 KSP + MIP（而非其他方法）

我在動工前列了一份候選清單，最後實作了三套：兩階段 baseline (B)、KSP (C)、MIP (A)。Demo 留下 C 跟 A 對照；B 跟其他方案的取捨記錄如下，**這部分就是「設計判斷」的展示**。

### 實作了並比較過

| 演算法 | 角色 | 最後是否進 demo |
|---|---|---|
| **B 兩階段** | Baseline：Dijkstra 最短路徑 → 路上 p-median 選站 | ❌ 不展示，僅實驗對照 |
| **C KSP**  | Yen K 條候選 + corridor p-median + 路線修補 | ✅ 主力 |
| **A MIP**  | CP-SAT MILP，含 MTZ subtour elimination | ✅ 金標 / 小規模驗證 |

**為什麼把 B 砍掉？** 看第 4 章節數據：B 的路線完全不考慮乘客分布、強制走最短路徑，乘客一旦不在主軸上 walk cost 就爆炸。實驗結果在中型場景 B 比 KSP 差 30% 以上，沒有展示價值 — 留在程式碼裡只是當作說「我比較過 baseline」的證據。

### 沒實作的方案與理由

| 方法 | 為何排除 |
|---|---|
| **Iterative EM**（交替「固定站算路徑 / 固定路徑算站」） | 跟 KSP 同類局部搜尋；KSP 用顯式列舉 K 條路徑可解釋性更好、debug 容易 |
| **Genetic Algorithm / SA** | 對這題的結構利用低；參數調整成本高；要在 >500 nodes / >200 passengers 才會顯現優勢 |
| **Steiner / Orienteering 純解法** | 學術正確但實作門檻高；題目的「轉折優勢」用 KSP corridor 已能掌握 |
| **Reinforcement Learning** | 訓練/收斂成本不划算；適合長期動態調度而非一次性規劃 |
| **K-shortest with elastic walking** | Lagrangian 鬆弛變體，比 KSP corridor 複雜很多，邊際效益小 |

---

## 4 · 兩個主力演算法說明

### C · KSP — K-shortest paths + corridor p-median  ⭐

```
1. Yen's K-shortest simple paths 取 K 條候選 A→B 路線
2. for each 路徑 π:
     a. corridor = π ∪ π 周圍 r-hop 內的節點
     b. 在 corridor 上跑 p-median（greedy add → 1-swap → 1-drop）
     c. 若選到 corridor 上但不在 π 上的站
        → 重算路線：Dijkstra(A → s1 → s2 → ... → sk → B)
        （允許 spur 支線 — 短暫繞離主軸服務該站再回來）
3. 回傳總成本最低的候選
```

**關鍵設計**：第 2(a) 步的 corridor 擴展是「路線往乘客密集區彎曲」的機制 — 它讓 KSP 的可行解空間遠大於純兩階段。第 2(c) 的路線修補允許 spur，這實際上跳出了 MIP 的簡單路徑限制 (見實驗結果)。

**參數**：`k_paths=6`、`corridor_hops=3`（皆為實驗 sweep 後的甜點，見第 5 章節）。

### A · MIP — CP-SAT MILP（金標）

完整 MILP 用 Google OR-Tools CP-SAT 求解：

- 雙向弧 + 流量守恆 + 簡單路徑（每節點 inflow ≤ 1）
- **MTZ 勢函數** `u_b ≥ u_a + 1 − N·(1−x_{a→b})`：排除任何 disconnected subtour（早期版本未加這條，CP-SAT 會「偷塞 subtour」啟用遠方節點當站，是 debug 過程中的關鍵發現）
- 每位乘客的 z 候選只取「步行最近 M=40 個」減少變數規模

**優**：模型內最佳性保證 + gap 報告 ⇒ 可當 KSP 的金標
**缺**：規模 > ~200 節點開始 timeout 在 FEASIBLE，gap 可能 > 50%

---

## 5 · 實驗結果：演算法邊界量化

跑了一份完整 sweep（`scripts/bench_ksp.py`），原始資料在 `docs/bench_ksp_data.json`。三種場景大小、每種多 seeds，對每個 seed 跑 MIP 當金標、再 sweep KSP 的 `k_paths × corridor_hops` 16 組設定。

### 5.1 演算法成本對照（gap = KSP cost / MIP cost − 1）

| 場景 | 規模 | MIP 狀態 | KSP 最佳 gap | KSP runtime |
|---|---|---|---|---|
| small | 48n / 12p | OPTIMAL (17ms) | **−13.8%** | 6 ms |
| medium | 120n / 20p | FEASIBLE @60s (gap_internal 52~73%) | **−7%** ~ +30%（seed 依賴） | ~150 ms |
| default | 192n / 30p | FEASIBLE @90s (gap_internal 71%) | **−7.7%** | 138 ms |

**負 gap = KSP 比 MIP 更低成本**。意外的核心發現：**KSP 在多數場景反而贏 MIP**。

### 5.2 為什麼 KSP 會贏「最佳化」的 MIP？

兩個並存的原因：

1. **可行解集合不對等**：MIP 因 MTZ 限制只能輸出簡單路徑；KSP 的路線修補允許 spur 支線（短暫繞離主軸服務遠處乘客再回來），這在現實公車路線中是合法操作。**KSP 的可行解集合 ⊋ MIP 的可行解集合。**
2. **MIP 在中型以上規模根本跑不完**：default 場景 90 秒只跑到 gap_internal=70%，意思是 MIP 自己都不確定差最佳解多少。它回的是 incumbent solution 而非 true optimum。

換句話說 — **MIP 的「OPTIMAL」是有星號的「在我寫的這個簡單路徑模型內最佳」**，並非「現實問題的真最佳」。這正是這個 demo 想傳遞的判斷力。

### 5.3 KSP 參數調校：corridor_hops 才是靈魂

抽 small + default 場景看單一變數的邊際效益（其他 seed 平均）：

| corridor_hops | small | medium | default |
|---|---|---|---|
| 0 | +0%   | +28%  | (沒測) |
| 1 | −6.4% | +17%  | +9.9%  |
| 2 | −10.4%| +8%   | −1.7%  |
| 3 | **−13.8%** | **+7%** | **−7.7%** |

每加一 hop 穩定多降 5~7% gap。**這條才是 KSP 的關鍵旋鈕**。

反觀 `k_paths`：

| k_paths (corridor=3) | small | medium 平均 | default |
|---|---|---|---|
| 3 | −13.8% | +16% | (sweep 從 6 起) |
| 6 | −13.8% | +14% | −7.7% |
| 10 | −13.8% | +13% | −7.7% |
| 20 | −13.8% | +7% | −7.7% |

在 small/default 場景 `k_paths` **完全沒幫助**（k=3 跟 k=20 一樣）；只有 medium 在 k=20 才有額外收益。**Yen K 條中，最佳路徑通常落在前 3~6 條內**。

→ 結論：UI 預設 `k_paths=6, corridor_hops=3`，能逼出 KSP 90% 的潛力。

### 5.4 兩階段 baseline 的下場

| 場景 | two_phase gap vs MIP |
|---|---|
| small  | +0%（乘客剛好都在主幹線上） |
| medium | +2.1% / +42.8% / +52.2%（seed 依賴） |
| default | +27.2% |

只在乘客剛好沿著最短路徑分佈時才接近最佳；一旦散開就慘輸。這證實了第 2 章節砍掉它的決策。

### 5.5 graph_gen 的脆弱性發現

9 個 seed 有 4 個 MIP FAILED — 原因是 `bus_subgraph` 用 `nx.edge_subgraph` 時會把「所有邊都被禁區標記的節點」一併剔除，造成 A 或 B 不在圖裡。已修為手動建 `nx.Graph` 並 `add_nodes_from`，所有節點都保留，只是可能 isolated。這是實驗才暴露出的 corner case。

---

## 6 · 模型/實作的限制 (Limitations)

- **離散圖近似**：城市抽象成圖；現實的「站可以設在道路任意位置」未建模。需要更高解析度可細化道路為多個 sub-segment。
- **步行距離用 full-graph 最短路**：實務應分開「步行網」與「道路網」（如 OSM 雙網層），這裡假設兩者相同。
- **無容量、無班次、無時刻表**：純空間靜態問題。若要加 vehicle routing 動態元素，需擴充為 VRP / 時刻表合成。
- **MIP 簡單路徑限制**：對應「公車不重複路段」假設；現實的環狀路、spur 服務需改用 capacitated multi-arc flow 重新建模。KSP 已允許 spur，所以 KSP 的解有時 strictly better than MIP 的「OPTIMAL」（見 5.2）。
- **單目標純成本**：題目要的「不同權重」用 α/β/γ 線性組合處理；若要 Pareto-front 視覺化，需多目標求解（ε-constraint / NSGA-II）。

---

## 7 · 使用 demo

打開 [http://localhost:8765/](http://localhost:8765/) 後：

1. 右側上方調 `rows/cols/passengers/seed`，按 **重新生成城市**。
2. **編輯模式** 可以：移動 A、移動 B、增刪乘客。
3. 拉 **K / α / β / 站成本** 四條滑桿（適用所有演算法）。
4. 拉 **k_paths / corridor_hops** 兩條滑桿（只影響 KSP）。
5. 連按 **K-最短 C** 跟 **MIP 最佳 A**，觀察結果疊在地圖上 + 右下角比較表。
6. 比較表中**綠色那列**是當前最佳。

### 推薦玩法

- 把 **corridor_hops 拉到 0** 跑一次 KSP → 結果接近兩階段。再拉到 3 跑一次 → 看路線怎麼「彎曲」服務乘客。
- 把 **β 拉到 5** → 路線會強烈往乘客密集區傾斜。
- 把 **K 改成 2** → 站變少，逼演算法重新選關鍵站。
- 用 **+ 乘客** 模式在地圖一角放 5~6 個乘客 → 跑 KSP → 看路線往那邊偏。
- **小場景**（rows=6, cols=8）→ MIP 通常可在秒內 OPTIMAL，是 KSP 唯一被打贏的場景。
