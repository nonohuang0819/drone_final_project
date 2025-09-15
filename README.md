# Drone Final Project

## 專案簡介
這是一個無人機相關的專案，使用Python和ROS進行開發。

## 專案協作規定

### 1. Git 工作流程

#### 分支管理
- `main`: 主分支，僅用於穩定的發布版本
- `develop`: 開發分支，所有新功能開發的基礎分支
- `feature/*`: 功能分支，用於開發新功能
- `hotfix/*`: 修復分支，用於緊急修復生產環境問題
- `release/*`: 發布分支，用於準備新版本發布

#### 分支命名規範
```
feature/功能名稱-簡短描述
hotfix/問題編號-簡短描述
```

範例：
```
feature/flight-controller-integration
hotfix/001-sensor-data-crash
release/v1.2.0
```

### 2. Pull Request (PR) 規範

#### PR 標題格式
```
[類型] 簡短描述

類型包括：
- feat: 新功能
- fix: 修復Bug
- docs: 文檔更新
- style: 代碼格式調整（不影響功能）
- refactor: 重構代碼
```

範例：
```
[feat] 添加無人機自動巡航功能
[fix] 修復感測器數據讀取異常
[docs] 更新API文檔和使用說明
```

#### PR 描述模板
```markdown
## 變更描述
簡要描述這次變更的內容和目的


## 額外說明
其他需要說明的內容
```


