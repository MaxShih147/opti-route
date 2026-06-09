# opti-route

> 公車主路線設計 + 站點選址的**聯合最佳化**互動 demo。把兩個傳統上分開處理的問題建模為單一耦合問題，用兩種風格的解法並排比較。

**🌐 Live demo:** [opti-route.max-the-solution.com](https://opti-route.max-the-solution.com/)

---

## 為什麼有這個專案

我對「兩個經典圖論問題糾纏在一起會冒出什麼新解空間」很好奇：

- **A → B 最短路徑**（with detours, forbidden zones, terrain cost）
- **p-median facility location**（K 個站、最小化乘客總步行）

單獨來看都是 textbook material。但合在一起：路線形狀**會被站點位置牽動**、站點選擇**反過來受路線形狀限制** —— 就變成 NP-hard 的耦合問題。

所以寫了這個 demo 同時實作**啟發式**跟**精確解**兩種風格，並排觀察它們在不同規模下的相對表現。

完整技術細節（建模、約束、debug 故事）請見 **[docs/ALGORITHMS.md](docs/ALGORITHMS.md)**。

---

## Quick start（本機跑）

```bash
git clone https://github.com/MaxShih147/opti-route
cd opti-route

/opt/homebrew/bin/python3.12 -m venv .venv
.venv/bin/pip install -r backend/requirements.txt
.venv/bin/uvicorn backend.main:app --port 8765

open http://localhost:8765/
```

需要 Python 3.12+（用了較新的 type hint 語法）。

---

## 介面總覽

```
┌──────────────────────┬──────────────────────────────────────────┐
│  opti-route   about  │  ● Ready · 192 nodes · 100 pax   [圖例▾] │
│  ────────────────    │                                          │
│  ▾ 城市生成          │                                          │
│   ┌────────┐ ┌────┐  │       ●  ●      ●●                       │
│   │城市規模│ │乘客│  │     ▦      ●     ●  ●●                   │
│   │   100  │ │100 │  │   ──┼──┼──┼──┼──┼──                     │
│   ├────────┴────┤    │   ┌──■A      ●      ■B┐                  │
│   │  生成城市   │    │   │              ╭───╯                   │
│   └─────────────┘    │   │              ○ (站)                   │
│                      │   ╰──○──○──○──● ●                         │
│  ▾ 成本參數          │                                          │
│  K=5 α=1.0 β=1.0 …   │                                          │
│                      │                                          │
│  ▾ 求解              │                                          │
│  [K] K-最短路徑啟發式│                                          │
│  [M] 混合整數規劃    │                                          │
│                      │                                          │
│  ▾ 結果比較          │                                          │
│   方法  總計  路線…  │                                          │
│   ◆ K   ……   ……     │                                          │
│   ◆ M   ……   ……     │                                          │
└──────────────────────┴──────────────────────────────────────────┘
   ← 控制面板             ← 地圖（SVG，互動可縮放）
```

手機版會自動把控制面板堆到上半部、地圖在下半部。

---

## 操作步驟

### 1 · 生成城市

| 欄位 | 意義 | 範圍 |
|---|---|---|
| **城市規模** | 路口總數，自動換算成 cols×rows 偏橫向 | 16–400 |
| **乘客點數** | 隨機散布，70% 集中於 hotspots、30% 均勻背景 | 20–500 |
| **畸零度** | 道路稀疏度（0% = 完整網格、20% = 很多斷頭） | 0–50 |
| **禁區數** | 不可穿越的阿米巴形區域數 | 0–6 |

按下「生成城市」會每次隨機種子。每個禁區自動限制：
- 覆蓋路口 ≤ 4（小規模）或 ≤ 8（大規模）
- 任兩禁區共享路口 = 0

### 2 · 觀察初始畫面

- 🟢 **A 起點**（深綠方塊） · 🔴 **B 終點**（紅）
- 🟢 **乘客**（淺綠圓點，大小代表 demand 1–6 人）
- 🌈 **地形漸層**（青色＝便宜、橘紅＝昂貴；公車經過貴的地方營運成本更高）
- 🟥 **禁區**（紅色虛線圈起來的 amoeba 形狀）
- 〰️ **道路**（細灰＝一般、粗灰＝快速道路）

### 3 · 調整成本參數

| 參數 | 數學符號 | 直觀 |
|---|---|---|
| 最多站數 | K | 預算上限，K 越大解空間越彈性 |
| 路線權重 | α | α 越大 → 演算法傾向短路徑 |
| 步行權重 | β | β 越大 → 演算法傾向繞遠路服務乘客 |
| 站建設成本 | c_fix | 每設一個站固定費（× 地形係數） |

預設 K=5, α=1.0, β=1.0, c_fix=50 已經是「平衡」的設定，建議先用這組看看。

### 4 · 求解

按 **[K] K-最短路徑啟發式** 或 **[M] 混合整數規劃** 任一顆。

- 第一次按 → 計算 + 渲染（K 通常 <500ms、M 通常 1–10s）
- 第二次按同一顆 → 從 cache 取結果、直接切換顯示（不重算）
- 點下方比較表的列也能切換顯示

求解後地圖會：
- 路線藍粗線
- 站牌橘色甜甜圈
- 乘客→站的步行虛線
- 背景元素（道路、地形、禁區）淡化讓主要結果突顯

### 5 · 看比較表

下方表格列出兩種方法：

```
方法   |        成本             | 運算時間
       | 總計  路線  設站  步行  |     ms
─────────────────────────────────────────
◆ K    | 26271 2163  314  23794 |    459
◆ M    | 25604 1985  297  23322 |   1635
```

◆ 標記是當前顯示在地圖上的方法。**綠底列**是當前場景下總成本最低者。

---

## 兩個解法（簡短版）

| 方法 | 類型 | 核心技術 | 規模 |
|---|---|---|---|
| **K-最短路徑啟發式** | Heuristic | Yen K-shortest + corridor p-median + path repair | 1000+ nodes |
| **混合整數規劃** | Exact | OR-Tools CP-SAT + MTZ subtour elimination | ~400 nodes (timeout) |

兩個都根據圖規模自動調參數（不需要手動）。

**有趣的觀察**：大場景下 KSP 反而會**勝過** MIP，因為 KSP 允許「支線繞行（spur）」、MIP 強制簡單路徑。這不是 bug —— 是兩個方法的可行解空間根本不同。

詳細討論看 [docs/ALGORITHMS.md §5–6](docs/ALGORITHMS.md)。

---

## 結果範例

### 中型場景（200 路口、100 乘客）

```
方法              總計     路線    設站    步行    ms
K-最短路徑啟發式  26271    2163    314    23794    459
混合整數規劃      25604    1985    297    23322   1635
```

MIP 找到稍微便宜的解，但 KSP 快 3.5×。

### 大型場景（400 路口、100 乘客）

```
方法              總計     路線    設站    步行       ms
K-最短路徑啟發式  64368    3712    287    60369      808
混合整數規劃      75764    4501    312    70951    30000+  (timeout)
```

KSP **大勝** MIP 15%。MIP 撞 30s timeout、回傳 incumbent 但 incumbent 沒到最佳。KSP 透過 spur 找到 MIP 表達不出的解。

---

## 技術棧

| | |
|---|---|
| **後端** | FastAPI · NetworkX · Google OR-Tools CP-SAT |
| **前端** | 純 SVG + vanilla JS（無 framework） |
| **地圖** | 程序化生成：擾化網格 + 主幹道 + 阿米巴禁區 + value noise 地形 |
| **部署** | Mac Studio M3 Ultra (28 core / 96GB RAM) + Cloudflare Tunnel + launchd auto-restart |
| **CI/CD** | uvicorn `--reload` + auto-pull script（push 1 分內生效） |

---

## 專案結構

```
opti-route/
├── backend/                     FastAPI + 演算法
│   ├── graph_gen.py             城市生成（網格、地形、禁區、乘客）
│   ├── models.py                Pydantic / dataclass shared types
│   ├── main.py                  /api endpoints + CORS / cache headers
│   ├── requirements.txt
│   └── algorithms/
│       ├── common.py            Dijkstra walking distances + p-median
│       ├── ksp.py               Yen K-shortest + corridor + path repair
│       ├── mip.py               CP-SAT MILP w/ MTZ subtour elimination
│       └── two_phase.py         baseline（演算法比較用，UI 不顯示）
│
├── frontend/                    純靜態 UI
│   ├── index.html               主頁
│   ├── about.html               關於頁
│   ├── styles.css               全部樣式（含 RWD）
│   ├── app.js                   控制 + 渲染 + 求解 dispatch
│   └── solver.js                純前端 KSP 移植（static-no-backend branch）
│
├── docs/
│   ├── ALGORITHMS.md            ⭐ 完整演算法詳述
│   ├── bench_ksp_data.json      KSP 參數 sweep 原始資料
│   └── Algorithm Assignment.pdf
│
├── deploy/                      生產部署
│   ├── DEPLOY.md                Mac Studio + Cloudflare 完整步驟
│   ├── auto-pull.sh             git pull → 重啟 backend
│   ├── cloudflared.yml          tunnel ingress
│   └── com.maxshih.opti-route.{backend,tunnel,auto-pull}.plist
│
└── scripts/
    └── bench_ksp.py             KSP × MIP 參數 sweep 腳本
```

---

## 開發 / 部署

**本機開發**：
```bash
.venv/bin/uvicorn backend.main:app --reload --port 8765
```
`--reload` 讓 backend `.py` 存檔後 1 秒內自動重載。前端是 static，瀏覽器重整就會看到。

**Benchmark 跑一輪**：
```bash
PYTHONPATH=. .venv/bin/python scripts/bench_ksp.py
```

**生產部署**：見 [deploy/DEPLOY.md](deploy/DEPLOY.md)。簡單講就是 launchd 跑 uvicorn + cloudflared，crash 自動重啟、push 1 分鐘內自動拉。

---

## 其他分支

- `main` — 後端 + 前端，當前生產版本
- `static-no-backend` — 全部 JS 化（含 KSP），可 deploy 到 Cloudflare Pages 純靜態，但 MIP 用 HiGHS-WASM 品質還不夠穩定

---

## License

MIT
