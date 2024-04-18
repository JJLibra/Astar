<p align="center">
    <a target="_blank" href="https://github.com/JJLibra/Astar">
        <img src="https://github.com/JJLibra/Astar/blob/main/README/RDME_IMG/purchase.png" alt="astar-logo" width="150" data-width="150" data-height="150">
    </a>
</p>

<h1 align="center">Astar算法演示器——無人機路徑規劃</h1>

<p align="center">
    <a target="_blank" href="https://github.com/JJLibra">
      <img style="display:inline-block;margin:0.2em;" alt="Author" src="https://img.shields.io/badge/Author-Junjie Li-blue.svg?logo=autoit&style=flat">
    </a>
    <a target="_blank" href="https://github.com/JJLibra/Astar">
      <img style="display:inline-block;margin:0.2em;" alt="GitHub Repo stars" src="https://img.shields.io/github/stars/JJLibra/Astar?style=social">
    </a>
    <a target="_blank" href="https://github.com/JJLibra/Astar">
      <img style="display:inline-block;margin:0.2em;" alt="Qt" src="https://img.shields.io/badge/Framework-Qt-green.svg?logo=Qt&style=flat">
    </a>
</p>

<p align="center">
    <a href="https://github.com/JJLibra/Astar">GitHub</a>
    &nbsp; | &nbsp;
    <a href="https://github.com/JJLibra/Astar/blob/main/README/README.zh_CN.md">简体中文文档</a>
    &nbsp; | &nbsp;
    <a href="https://github.com/JJLibra/Astar/blob/main/README/README.zh_TW.md">繁体中文文档</a>
    &nbsp; | &nbsp;
    <a href="https://github.com/JJLibra/Astar/blob/main/README/README.de_DE.md">德语文档</a>
</p>

## 目錄

[運行界面](#運行界面)

[安裝與打包](#安裝與打包)

[註意事項](#註意事項)

[功能介紹](#功能介紹)

[演示](#部分演示)

[其他說明](#其他說明)

## 運行界面

![界面](./RDME_IMG/界面.jpg)

## 註意事項

- 項目路徑務必都為英文，否則構建時會報錯
- 首次運行時圖標可能會無法顯示，應該是未成功導入，解決方法：
QT creator中打開項目以後，在`資源`-`icon.qrc`-`添加現有文件`重新添加圖標文件即可。

## 安裝與打包

本項目基於 QT 框架使用 C++ 開發

開發環境：Windows

```
git clone git@github.com:JJLibra/A-star.git
```

建議安裝 QT creator 打開 Astar.pro 文件即可編輯

我使用的是 Enigma Virtual Box 打包，可以[參考這篇博客](https://blog.csdn.net/qq_40994692/article/details/113880198)

## 功能介紹

### 界面

```
1. 自定義地圖寬高
2. 地圖單元格大小可調
3. 自定義起點、終點、障礙物
4. 底部狀態欄，便於調試
5. 頂部工具欄可隱藏
6. 顯示最優路徑的同時將探索點用不同顏色顯示在地圖中
7. 繪製地圖可以 .Amap 文件保存到本地文件夾
8. 可將保存的 .Amap 文件載入地圖
9. 性能分析，數據可視化處理
10. 生成隨機地圖（生成邏輯待優化，目前還不能確保生成的地圖一定存在可行路徑）
11. 可導入本地圖片作為地圖背景，一種偽柵格化操作
12. 「關於我們」頁面
13. 一點拙劣的 QSS 美化
```

### 算法

```
1. 深度優先搜索算法
2. 廣度優先搜索算法
3. Dijkstra 算法
4. 最佳優先搜索算法
5. 傳統 A 星算法
6. 雙向 A 星算法
7. 優化 A 星算法
   - 三種距離計算定義預估距離 h
      - 切比雪夫距離
      - 曼哈頓距離
      - 歐幾裏得距離
   - 整體動態加權 dynamic
   - 自定義拐角權值 penalty
   - 自定義安全距離模式 alpha
8. 三種增量式搜索算法（還不夠完善，不建議使用）
```

## 部分演示

- 自定義地圖參數

本項目可以自由設置地圖的長寬，單元格有四種狀態選擇：

![設置地圖](./RDME_IMG/Astar_gif/设置地图.gif)

- 優化Astar算法

其中一種優化A星算法的演示，且本項目支持采用**貝塞爾曲線**作為無人機路徑軌跡：

![Astar算法演示](./RDME_IMG/Astar_gif/A星&贝塞尔.gif)

- 生成隨機地圖 & 重置地圖數據

支持生成隨機地圖（生成邏輯比較簡單，利用偽隨機數生成，二次開發時可以優化這裏的生成邏輯）和一鍵重置地圖參數：

![隨機地圖](./RDME_IMG/Astar_gif/随机地图.gif)

- 保存地圖到本地

考慮到項目的展示環節，不可能現場畫地圖，所以本項目支持將地圖保存到本地：

![保存地圖](./RDME_IMG/Astar_gif/保存地图.gif)

- 打開本地地圖 & 深度優先算法演示

打開保存的.Amap文件，演示深度優先算法，按下`深搜最短`可以獲得所有深搜得到路徑中的最短路徑（深搜實現沒有問題，只是操作時需要註意，一步步來，否則容易出bug）：

![打開地圖](./RDME_IMG/Astar_gif/深搜.gif)

- 導入背景圖片

導入圖片作為背景，描繪地圖（二次開發時建議開發柵格化處理）：

![背景圖片](./RDME_IMG/Astar_gif/打开&清除背景.gif)

- 其他

還有不少小細節（關於頁、可視化性能分析、操作提示框、頁腳狀態欄、進入的淡入淡出...）：

![其他](./RDME_IMG/Astar_gif/其他.gif)

## 其他說明

項目用於學校的課設，所以一些 ico 圖標都與 NWPU 相關。另外，項目製作時還是有很多不規範的地方，小細節有不少但是bug也不少，還有點史山...大家多多包涵。

[返回顶部](#A-star)