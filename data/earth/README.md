# 地球场景数据配置

## 方案选择（针对中国境内使用）

### 1. 国内在线服务方案（推荐）
**文件：`china_online.earth`**
- 使用高德地图卫星影像，国内访问速度快
- 包含卫星影像和道路标注
- 无需下载大型数据文件
- 首次加载需要网络，后续会缓存

### 2. 离线全球方案
**文件：`offline_global.earth`**
- 完全离线，无需网络
- 使用 Natural Earth NE2 Cross-blended Hypsometric Tints 纹理
- 适合无网络环境使用
- 需要下载全球纹理文件（见下方）

### 3. 国际在线服务方案
**文件：`highres_global.earth`**
- 使用 ArcGIS World Imagery
- 全球覆盖，分辨率高
- 在国内可能访问受限或速度慢

## 离线方案准备

### 下载全球纹理文件
1. 下载 Natural Earth NE2 Cross-blended Hypsometric Tints 文件：
   - 下载链接：[NE2 Cross-blended Hypsometric Tints](https://www.naturalearthdata.com/downloads/50m-raster-data/50m-natural-earth-2/)
   - 选择 "HYP_50M_SR_W.zip" 下载

2. 解压缩文件，将以下文件放置到 `data/earth/` 目录：
   - `HYP_50M_SR_W.tif`（主要纹理文件）
   - `HYP_50M_SR_W.prj`（投影文件，可选但推荐）
   - `HYP_50M_SR_W.aux.xml`（辅助文件，可选）

### 可选：地形数据
如果需要地形效果，可以下载 SRTM 数据：
- 推荐：[SRTM 90m Digital Elevation Database](https://cgiarcsi.community/data/srtm-90m-digital-elevation-database-v4-1/)
- 下载后重命名为 `srtm.tif` 并放置到 `data/earth/` 目录

## 使用方法

1. **使用国内在线服务**（推荐）：
   - 重命名 `china_online.earth` 为 `highres_global.earth`
   - 确保网络连接正常

2. **使用离线方案**：
   - 下载并放置 Natural Earth 纹理文件到 `data/earth/` 目录
   - 重命名 `offline_global.earth` 为 `highres_global.earth`

3. **程序会自动检测**：
   - 优先使用 `highres_global.earth`
   - 如果加载失败，会尝试在线备用方案
   - 如果所有在线方案都失败，会使用程序化纹理

## 缓存设置
- 在线服务会缓存瓦片到 `./osgearth_cache` 目录
- 首次加载较慢，后续运行会加速
- 缓存目录可安全删除，程序会重新创建

## 技术说明
- **在线方案**：适合有网络环境，实时获取最新地图数据
- **离线方案**：适合无网络环境，或对网络速度有要求的场景
- **分辨率**：在线服务可达 0.5m，离线方案为 50m 分辨率
- **Natural Earth 纹理特点**：
  - 专为地形可视化优化
  - 包含陆地、海洋、冰川等地理特征
  - 50m 分辨率全球覆盖
  - 自带地形高度着色

## 故障排除
- 如果显示简化地球：检查文件路径是否正确
- 如果加载缓慢：清理缓存目录重新加载
- 如果出现白屏：检查文件权限和完整性
