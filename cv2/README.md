## 🌟 以下是各個 test 的解釋，裡面的 subsrciption, publisher 要改成我們課堂上能跑的那幾個

### `test1.py` - 循序任務飛行

#### 功能說明
此腳本讓 Tello 無人機執行一個預先定義好的、基於時間的飛行任務序列。

#### 運作方式
程式碼中的 `TelloController` 類別包含一個名為 `task_flow` 的列表。程式會依序執行這個列表中的每一個動作，直到所有動作都完成為止。每個動作都包含一個名稱和持續時間。

#### 如何指定任務
要修改飛行任務，請直接編輯 `test1.py` 中 `TelloController` 類別下的 `self.task_flow` 列表 (約在第 40 行)。

可用的動作 (`name`) 包括：
- `takeoff`: 起飛
- `land`: 降落
- `ascend`: 爬升
- `forward`: 前進
- `rotate`: 旋轉
- `hover`: 懸停

**範例：**
若要讓無人機起飛後，向前飛 5 秒，然後降落，可以將 `task_flow` 修改為，修改這個 list 就能有不同動作：
```python
self.task_flow = [
    {"name": "takeoff", "duration": 2.0},
    {"name": "forward", "duration": 5.0},
    {"name": "land",    "duration": 2.0},
]
```

---

### `test2.py` - 顏色偵測與視覺化


#### 運作方式
它會訂閱 Tello 的影像串流，並使用 OpenCV 進行顏色空間轉換 (BGR to HSV)。透過一個名為 "controls" 的 GUI 視窗，您可以使用滑桿即時調整 HSV 的門檻值，以篩選出您想偵測的顏色。程式會找到符合顏色範圍的最大物體，並在其周圍繪製一個綠色的邊界框。

#### 如何指定任務
這個程式的主要用途是**調整顏色偵測的參數**。執行此腳本後，請觀察 `mask` 和 `res` 視窗，並拖動 `controls` 視窗中的滑桿，直到您想追蹤的物體在 `mask` 視窗中呈現為白色，而在 `res` 視窗中被綠色框準確框選。記下此時的 `low H/S/V` 和 `high H/S/V` 值，這些值可以用於 `test3.py` 的自動飛行任務中。

---

### `test3.py` - 視覺伺服 (Visual Servoing) 飛行

#### 功能說明
此腳本結合了電腦視覺和飛行控制，讓無人機能夠自主地尋找、對齊一個特定顏色的物體，然後飛越它並降落。

#### 運作方式
1.  **影像處理**: 如同 `test2.py`，它會根據 HSV 門檻值來偵測物體。但演算法更進階，它會優先選擇更像矩形的輪廓，並計算出物體的中心點座標 `(cx, cy)`。
2.  **通訊**: 節點會將計算出的中心點座標和一個 `can_pass` 旗標 (表示是否已對準) 發佈到 `/ip_inform` 主題。
3.  **飛行控制**:
    - 腳本內建一個簡單的狀態機 (`task_index`)，依序執行 `takeoff` -> `align & pass` -> `land`。
    - 在 `align & pass` 階段，它會訂閱 `/ip_inform` 的資訊。
    - 它會計算物體中心與畫面中心的誤差 `(dx, dy)`，並產生對應的 `Twist` 指令來移動無人機，使誤差變小。
    - 當誤差足夠小 (`can_pass == 1`)，表示已對準，無人機便會向前飛行一小段距離，然後進入降落階段。

#### 如何指定任務
此腳本的任務是**視覺追蹤和穿越**，其行為主要由**顏色門檻值**決定。
1.  **設定顏色**: 在 `process_and_draw` 函式中，您需要手動設定 `hsv_low` 和 `hsv_high` 的值。這些值可以透過執行 `test2.py` 來找到。
    ```python
    # 在 process_and_draw 函式中
    H_low  = cv2.getTrackbarPos('low H',  'controls')
    # ...
    # 您可以將這些 getTrackbarPos 的值直接換成您用 test2.py 調好的數值
    # 例如:
    # hsv_low  = np.array([20, 100, 100], dtype=np.uint8)
    # hsv_high = np.array([30, 255, 255], dtype=np.uint8)
    ```
2.  **調整控制參數**: 在 `control_timer_callback` 函式中，您可以微調無人機的反應速度，例如修正對齊時的移動速度 (`msg.linear.y`, `msg.linear.z`) 或前進時的速度 (`msg.linear.x`)。


### Running steps
1. ``` colcon build```
2. ``` source install/setup.bash```
3. ``` ros2 launch launch.py```
4. 不同視窗：``` ros2 run tello ipn```