# GIS 相关基础知识

本文系统整理 GIS 开发中需要掌握的核心概念，包括地图投影、坐标系、瓦片系统、数据格式、地图服务等。

---

## 一、地图投影（Map Projection）

### 1.1 什么是地图投影

地球是一个不规则的三维椭球体，地图投影就是将地球表面的经纬度坐标映射到二维平面上的数学方法。**没有任何投影可以同时保持面积、形状、距离和方向都不变形**，选择投影就是选择保留哪种属性、牺牲哪种属性。

### 1.2 投影的分类

**按投影面分类：**

| 类型 | 说明 | 典型投影 |
|------|------|----------|
| **圆柱投影（Cylindrical）** | 将地球投影到圆柱面上再展开 | 墨卡托投影、Web Mercator、等角圆柱投影 |
| **圆锥投影（Conic）** | 将地球投影到圆锥面上再展开 | 兰伯特等角圆锥投影、Albers 等积圆锥投影 |
| **方位投影（Azimuthal）** | 将地球投影到平面上 | 极射赤面投影、兰伯特等积方位投影 |

**按变形性质分类：**

| 类型 | 说明 | 适用场景 |
|------|------|----------|
| **等角投影（Conformal）** | 保持角度/形状不变，面积变形 | 航海图、气象图（墨卡托） |
| **等积投影（Equal-Area）** | 保持面积不变，形状变形 | 统计分析、面积比较（Albers、Mollweide） |
| **等距投影（Equidistant）** | 保持距离不变 | 距离测量（方位等距投影） |
| **任意投影（Compromise）** | 各方面适度变形，整体视觉效果好 | 世界地图展示（Robinson） |

### 1.3 Web Mercator vs 地理坐标（EPSG:3857 vs EPSG:4326）

| 特性 | Web Mercator（EPSG:3857） | 地理坐标（EPSG:4326） |
|------|--------------------------|----------------------|
| **本质** | 投影坐标系，单位为米 | 地理坐标系，单位为度 |
| **坐标形式** | 平面 X/Y（米） | 经度/纬度（度） |
| **面积变形** | 高纬度地区面积严重放大（格陵兰看起来和非洲一样大） | — |
| **适用场景** | Web 地图瓦片切片、快速渲染 | GPS 数据、数据存储、空间查询 |
| **Y 轴范围** | ±20037508.3427892 米 | ±90° 纬度 |
| **X 轴范围** | ±20037508.3427892 米 | ±180° 经度 |

**为什么 Web 地图偏爱 Web Mercator？**

- Google 工程师简化了标准墨卡托投影的公式，将地球视为正球体而非椭球体，计算更快
- 投影后地图是正方形，天然适合瓦片切割（2^n × 2^n）
- 等角特性使局部形状保持正确，导航时方向不偏
- 已成为事实标准：Google Maps、高德、百度、天地图等都采用

### 1.4 为什么投影很重要

- **数据叠加**：不同投影的数据无法直接叠加显示，必须统一投影
- **面积计算**：在 Web Mercator 下算面积会严重失真，高纬度地区需转回等积投影
- **距离计算**：投影坐标系下欧几里得距离 ≠ 球面距离
- **前端实战**：OpenLayers 加载高德地图出现拉伸，就是因为 View 投影与数据投影不匹配

---

## 二、坐标系（Coordinate System）

### 2.1 地理坐标系 vs 投影坐标系

```
地理坐标系（GCS）                    投影坐标系（PCS）
├── 定义：用经纬度描述位置            ├── 定义：用 X/Y（米）描述位置
├── 基于椭球体 + 大地基准面           ├── 基于：地理坐标系 + 投影方法
├── 单位：度（°）                     ├── 单位：米（m）
└── 例子：WGS84, CGCS2000             └── 例子：Web Mercator, UTM, 高斯-克吕格
```

### 2.2 WGS84（EPSG:4326）

- **全称**：World Geodetic System 1984
- **类型**：地心地固坐标系（ECEF），坐标原点在地球质心
- **参考椭球**：WGS84 椭球（长半轴 a = 6378137.0 m，扁率 f = 1/298.257223563）
- **用途**：GPS 卫星定位的默认坐标系，全球通用
- **特点**：无偏移，是"真实"的地理坐标

### 2.3 CGCS2000（EPSG:4490）

- **全称**：China Geodetic Coordinate System 2000（中国大地坐标系2000）
- **类型**：地心地固坐标系
- **参考椭球**：CGCS2000 椭球（长半轴 a = 6378137.0 m，扁率 f = 1/298.257222101）
- **与 WGS84 的关系**：
  - 原点、尺度、定向均相同，都属于地心地固坐标系
  - 椭球扁率仅有微小差异，引起同一点的坐标差异 **< 0.105mm**
  - **在绝大多数前端应用中，CGCS2000 与 WGS84 可视为等同**
  - 严格意义上，两者实现框架不同（WGS84 依赖 GPS 跟踪站，CGCS2000 依赖中国 CORS 站）

### 2.4 GCJ-02（火星坐标系）

- **全称**：国测局02坐标系，别称"火星坐标系"
- **性质**：在 WGS84 基础上的**加密偏移**坐标系
- **偏移量**：约 50~700 米，非线性，国境线附近呈强非线性特征
- **使用方**：高德地图、腾讯地图、Google 中国地图
- **法规背景**：中国《测绘法》规定，国内地图服务必须至少使用 GCJ-02 或在其基础上再次加密的坐标系，不得直接使用 WGS84 坐标发布地图

### 2.5 BD-09（百度坐标系）

- **性质**：在 GCJ-02 基础上的**二次加密**
- **使用方**：百度地图
- **加密方式**：引入额外的极坐标系旋转和平移变换
- **偏移量**：比 GCJ-02 多偏移约几十到几百米
- **逆向难度**：比 GCJ-02 更难逆向还原

### 2.6 坐标系对照表

| 坐标系 | EPSG | 性质 | 使用方 | 精度 |
|--------|------|------|--------|------|
| WGS84 | 4326 | 地理坐标系，无偏移 | GPS、国际标准 | 真实坐标 |
| CGCS2000 | 4490 | 地理坐标系，无偏移 | 中国国家测绘 | ≈ WGS84 |
| GCJ-02 | — | 加密偏移 | 高德、腾讯、Google中国 | 偏移50~700m |
| BD-09 | — | 二次加密偏移 | 百度地图 | 偏移更大 |
| Web Mercator | 3857 | 投影坐标系 | Web 地图瓦片 | 投影变形 |

### 2.7 坐标系转换关系链

```
WGS84 ──加密偏移──→ GCJ-02 ──二次加密──→ BD-09
  ↑                    │                     │
  │                    │                     │
  └──逆向(近似)────────┘──逆向(近似)─────────┘
```

> **注意**：GCJ-02 → WGS84 的逆转换是近似算法，因为偏移是非线性的。常见做法是迭代逼近或使用经验公式。

---

## 三、坐标格式

### 3.1 三种主要格式

| 格式 | 缩写 | 示例 | 说明 |
|------|------|------|------|
| **十进制度** | DD | `39.9042° N, 116.4074° E` | 计算机最常用，GIS 软件默认 |
| **度分** | DM | `39° 54.252' N` | 航海常用 |
| **度分秒** | DMS | `39° 54' 15.12" N` | 传统测绘、导航设备常用 |

### 3.2 转换公式

**DMS → DD：**
```
DD = 度 + 分/60 + 秒/3600
示例：39°54'15.12" = 39 + 54/60 + 15.12/3600 = 39.904200°
```

**DD → DMS：**
```
度 = floor(DD)
分 = floor((DD - 度) × 60)
秒 = ((DD - 度) × 60 - 分) × 60
示例：39.904200° → 39° 54' 15.12"
```

### 3.3 前端开发注意事项

- **GeoJSON 标准要求使用 DD 格式**，坐标顺序为 `[经度, 纬度]`
- GPS 设备输出通常为 DMS 或 DM
- 中国测绘数据常使用度分秒格式
- 前端库（如 `coordtransform`）可处理格式转换

---

## 四、瓦片系统（Tile System）

### 4.1 什么是瓦片

将地图按不同缩放级别切割成若干行列的矩形图像块（通常 256×256 像素），每一层级的瓦片数量呈 **4 的幂次增长**，形成瓦片金字塔。

### 4.2 缩放级别（Zoom Level）

| Zoom | 瓦片数（单边） | 总瓦片数 | 分辨率（m/pixel） | 比例尺（约） |
|------|---------------|---------|-------------------|-------------|
| 0 | 1 | 1 | 156543.03 | 1:5.9亿 |
| 1 | 2 | 4 | 78271.52 | 1:2.95亿 |
| 5 | 32 | 1024 | 4891.97 | 1:1840万 |
| 10 | 1024 | 1,048,576 | 152.87 | 1:57.5万 |
| 15 | 32768 | 1,073,741,824 | 4.78 | 1:1.8万 |
| 18 | 262144 | 68,719,476,736 | 0.60 | 1:2240 |
| 22 | 4194304 | — | 0.037 | 1:140 |

### 4.3 三种瓦片服务规范

| 特性 | XYZ（Slippy Map） | TMS | WMTS |
|------|-------------------|-----|------|
| **标准** | 非正式社区标准 | OSGeo 标准 | OGC 标准 |
| **URL 格式** | `/{z}/{x}/{y}.png` | `/{z}/{x}/{y}.png` | 复杂 KVP/RESTful |
| **Y 轴方向** | **从上往下**（Y=0 在北极） | **从下往上**（Y=0 在南极） | 由 TileMatrixSet 定义 |
| **适用场景** | OpenStreetMap、Google、高德 | 早期 OSGeo 项目 | 企业级 GIS（ArcGIS、GeoServer） |
| **复杂度** | 最简单 | 简单 | 最复杂（支持多 TileMatrixSet） |

**XYZ 与 TMS 的 Y 轴关系：**
```
TMS_Y = 2^Z - 1 - XYZ_Y
```

### 4.4 瓦片金字塔

```
         Zoom 0: 1×1 = 1 tile
        /              \
   Zoom 1: 2×2 = 4 tiles
      /                    \
 Zoom 2: 4×4 = 16 tiles
  /                          \
Zoom N: 2^N × 2^N = 4^N tiles
```

### 4.5 瓦片坐标计算（Web Mercator）

```javascript
// 经纬度 → 瓦片坐标
x = Math.floor((lon + 180) / 360 * Math.pow(2, z));
y = Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, z));
```

---

## 五、矢量数据格式

### 5.1 格式对比

| 格式 | 类型 | 编码 | 拓扑 | Web 友好 | 典型用途 |
|------|------|------|------|---------|---------|
| **GeoJSON** | 矢量 | JSON 文本 | 无 | ⭐⭐⭐⭐⭐ | Web 地图数据交换标准 |
| **TopoJSON** | 矢量 | JSON 文本 | **有** | ⭐⭐⭐⭐⭐ | GeoJSON 的拓扑编码扩展，体积更小 |
| **KML** | 矢量 | XML 文本 | 无 | ⭐⭐⭐ | Google Earth 生态 |
| **Shapefile** | 矢量 | 二进制（多文件） | 有 | ⭐ | 桌面 GIS 标准（ArcGIS/QGIS） |
| **WKT** | 矢量 | 纯文本 | 无 | ⭐⭐⭐ | 空间数据库、PostGIS |
| **WKB** | 矢量 | 二进制 | 无 | ⭐ | 空间数据库内部存储 |

### 5.2 GeoJSON 详解

```json
{
  "type": "Feature",
  "geometry": {
    "type": "Point",
    "coordinates": [116.4074, 39.9042]
  },
  "properties": {
    "name": "北京天安门"
  }
}
```

- **坐标顺序**：`[经度, 纬度]`（GeoJSON 规范 RFC 7946）
- **几何类型**：Point、LineString、Polygon、MultiPoint、MultiLineString、MultiPolygon、GeometryCollection
- **注意**：很多国内数据源使用 `[纬度, 经度]` 顺序，加载时务必检查！

### 5.3 TopoJSON

- GeoJSON 的扩展，编码了**拓扑关系**（共享边界只存储一次）
- 文件体积通常比 GeoJSON 小 **80%**
- 需要客户端解码转换回 GeoJSON 才能渲染
- 前端库：`topojson-client`

### 5.4 KML

- Keyhole Markup Language，Google Earth 原生格式
- 基于 XML，支持样式、网络链接、3D 模型
- 前端需解析库（如 `toGeoJSON`）转为 GeoJSON

### 5.5 Shapefile

- 至少由 `.shp`、`.shx`、`.dbf` 三个文件组成
- 二进制格式，**前端不能直接使用**，需转换为 GeoJSON
- 工具：`shpjs`（JavaScript）、`ogr2ogr`（命令行）、[mapshaper](https://mapshaper.org/)（在线）

### 5.6 WKT

```
POINT(116.4074 39.9042)
LINESTRING(116.4 39.9, 116.5 39.95)
POLYGON((116.3 39.8, 116.5 39.8, 116.5 40.0, 116.3 40.0, 116.3 39.8))
```

- 坐标顺序：`经度 纬度`（空格分隔）
- 前端库：`wellknown`、`@turf/wkt`
- 常用于 PostGIS 空间查询

---

## 六、空间参考系统与 EPSG

### 6.1 EPSG 是什么

EPSG（European Petroleum Survey Group）维护的空间参考系统注册表，为每个坐标系分配唯一编码。**EPSG 编码 = 坐标系的身份证号**。

注册表地址：[https://epsg.io](https://epsg.io)

### 6.2 常见 EPSG 编码

| EPSG 编码 | 名称 | 类型 | 说明 |
|-----------|------|------|------|
| **4326** | WGS84 | 地理坐标系 | 全球通用，GPS 标准 |
| **3857** | WGS84 Web Mercator | 投影坐标系 | Web 地图标准 |
| **4490** | CGCS2000 | 地理坐标系 | 中国国家大地坐标系 |
| **4214** | Beijing 1954 | 地理坐标系 | 旧参心坐标系 |
| **4610** | Xian 1980 | 地理坐标系 | 旧参心坐标系 |
| **4547** | CGCS2000 3-degree GK Zone 39 | 投影坐标系 | 中国2000高斯投影 |
| **4548** | CGCS2000 3-degree GK Zone 40 | 投影坐标系 | 中国2000高斯投影 |
| **32650** | WGS84 UTM Zone 50N | 投影坐标系 | UTM 投影 |
| **3395** | WGS84 World Mercator | 投影坐标系 | 标准墨卡托（非Web） |

### 6.3 EPSG 在前端的使用

```javascript
// OpenLayers 中注册自定义 EPSG
import proj4 from 'proj4';
import { register } from 'ol/proj/proj4';

// 从 epsg.io 获取 Proj4 定义字符串
proj4.defs('EPSG:4490', '+proj=longlat +ellps=GRS80 +no_defs');
proj4.defs('EPSG:4548', '+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +no_defs');
register(proj4);
// 然后即可使用 'EPSG:4490'
```

---

## 七、地图服务

### 7.1 OGC 标准服务

| 服务 | 全称 | 返回数据 | 特点 |
|------|------|---------|------|
| **WMS** | Web Map Service | 栅格图像（PNG/JPEG） | 按需渲染，每次请求动态生成图片，支持自定义样式 |
| **WMTS** | Web Map Tile Service | 预渲染瓦片 | OGC 标准，缓存瓦片，性能好，支持 KVP/REST/SOAP |
| **WFS** | Web Feature Service | 矢量数据（GML/GeoJSON） | 可查询、可编辑，支持空间过滤 |
| **WCS** | Web Coverage Service | 栅格原始数据 | 返回像元值，用于遥感分析 |

### 7.2 非标准/社区服务

| 服务 | 说明 | URL 格式 |
|------|------|---------|
| **TMS** | OSGeo 瓦片标准，Y 轴从南向北 | `/{z}/{x}/{y}.png` |
| **XYZ** | 社区事实标准，Y 轴从北向南 | `/{z}/{x}/{y}.png` |
| **Vector Tiles** | 矢量瓦片（MVT/Protobuf） | `/{z}/{x}/{y}.pbf` |

### 7.3 栅格瓦片 vs 矢量瓦片

| 特性 | 栅格瓦片（Raster Tiles） | 矢量瓦片（Vector Tiles） |
|------|------------------------|------------------------|
| **格式** | PNG/JPEG | Protocol Buffers（.pbf） |
| **数据** | 像素 | 几何 + 属性 |
| **样式** | 服务端渲染，固定 | 客户端渲染，可自定义 |
| **体积** | 较大 | 小约 75% |
| **旋转/缩放** | 缩放模糊 | 无损缩放旋转 |
| **典型提供方** | 天地图、ArcGIS 缓存 | Mapbox、OpenMapTiles |

### 7.4 前端加载示例

```javascript
// OpenLayers 加载 WMTS（天地图）
import TileLayer from 'ol/layer/Tile';
import WMTS from 'ol/source/WMTS';

new TileLayer({
  source: new WMTS({
    url: 'http://t0.tianditu.gov.cn/vec_c/wmts?tk=您的密钥',
    layer: 'vec',
    matrixSet: 'c',
    format: 'tiles',
    // ... TileGrid 配置
  })
});

// OpenLayers 加载 XYZ（高德）
import XYZ from 'ol/source/XYZ';

new TileLayer({
  source: new XYZ({
    url: 'https://webrd0{1-4}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}'
  })
});
```

---

## 八、大地基准面（Geodetic Datum）

### 8.1 什么是大地基准面

大地基准面是**最密合局部或全部大地水准面的数学模式**，由两部分定义：

1. **参考椭球体**：定义地球的形状和大小（长半轴 a、短半轴 b / 扁率 f）
2. **椭球定位与定向**：椭球体相对于地球的位置关系（平移、旋转、缩放，共 6 个参数）

> 形象比喻：地球 ≈ 表面坑洼的马铃薯；参考椭球体 ≈ 形状规则的鸭蛋；大地基准面 ≈ 将鸭蛋放在马铃薯上，通过平移、旋转、缩放使其在某区域最贴合

### 8.2 大地基准面的分类

| 类型 | 说明 | 例子 |
|------|------|------|
| **地心基准面** | 椭球中心与地球质心重合，全球拟合 | WGS84、CGCS2000 |
| **参心基准面** | 椭球中心偏离地球质心，局部区域最佳拟合 | Beijing 1954、Xi'an 1980 |

### 8.3 为什么基准面很重要

- **同一经纬度，不同基准面，地面位置不同**：北京54 的 (116°E, 40°N) 和 WGS84 的 (116°E, 40°N) 对应的实地位置可能差几十到上百米
- **数据叠加的前提**：不同基准面的数据必须先统一基准面才能叠加
- **坐标系 ≠ 基准面**：两个坐标系可以使用相同的椭球体但不同的基准面

### 8.4 层级关系

```
空间参考系统（CRS）
├── 地理坐标系（GCS）
│   ├── 参考椭球体（Ellipsoid）── 定义形状
│   └── 大地基准面（Datum）──── 定义位置
└── 投影坐标系（PCS）
    ├── 地理坐标系（GCS）─────── 基础
    └── 投影方法（Projection）─── 映射方式
```

---

## 九、中国坐标转换（火星坐标系）

### 9.1 为什么中国要求坐标偏移

- **法律依据**：《中华人民共和国测绘法》和《公开地图内容表示若干规定》
- **安全考量**：防止精确地理坐标被用于军事或安全威胁
- **规定**：在中国境内发布地图必须使用 GCJ-02 或在其基础上的加密坐标系
- **影响**：所有在中国运营的地图服务商（高德、百度、腾讯、Google中国）都必须遵守

### 9.2 各地图服务商使用的坐标系

| 地图服务商 | 坐标系 | 获取真实坐标的方式 |
|-----------|--------|------------------|
| **高德地图** | GCJ-02 | 需逆向转换 |
| **腾讯地图** | GCJ-02 | 需逆向转换 |
| **Google 中国** | GCJ-02 | 需逆向转换 |
| **百度地图** | BD-09 | 需两次逆向转换 |
| **天地图** | CGCS2000（≈ WGS84） | 直接可用 |
| **GPS 设备** | WGS84 | 原始坐标 |

### 9.3 转换算法（JavaScript）

```javascript
// 判断是否在中国境内
function isInChina(lng, lat) {
  return (lng > 73.66 && lng < 135.05 && lat > 3.86 && lat < 53.55);
}

// WGS84 → GCJ-02
function wgs84ToGcj02(lng, lat) {
  if (!isInChina(lng, lat)) return [lng, lat]; // 境外不偏移
  const a = 6378245.0; // 克拉索夫斯基椭球长半轴
  const ee = 0.00669342162296594323; // 偏心率平方
  let dLat = transformLat(lng - 105.0, lat - 35.0);
  let dLng = transformLng(lng - 105.0, lat - 35.0);
  const radLat = lat / 180.0 * Math.PI;
  let magic = Math.sin(radLat);
  magic = 1 - ee * magic * magic;
  const sqrtMagic = Math.sqrt(magic);
  dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * Math.PI);
  dLng = (dLng * 180.0) / (a / sqrtMagic * Math.cos(radLat) * Math.PI);
  return [lng + dLng, lat + dLat];
}

// GCJ-02 → BD-09
function gcj02ToBd09(lng, lat) {
  const x_pi = Math.PI * 3000.0 / 180.0;
  const z = Math.sqrt(lng * lng + lat * lat) + 0.00002 * Math.sin(lat * x_pi);
  const theta = Math.atan2(lat, lng) + 0.000003 * Math.cos(lng * x_pi);
  return [z * Math.cos(theta) + 0.0065, z * Math.sin(theta) + 0.006];
}

// BD-09 → GCJ-02
function bd09ToGcj02(lng, lat) {
  const x_pi = Math.PI * 3000.0 / 180.0;
  const x = lng - 0.0065;
  const y = lat - 0.006;
  const z = Math.sqrt(x * x + y * y) - 0.00002 * Math.sin(y * x_pi);
  const theta = Math.atan2(y, x) - 0.000003 * Math.cos(x * x_pi);
  return [z * Math.cos(theta), z * Math.sin(theta)];
}

// GCJ-02 → WGS84（近似逆运算，迭代法更精确）
function gcj02ToWgs84(lng, lat) {
  if (!isInChina(lng, lat)) return [lng, lat];
  let wgsLng = lng, wgsLat = lat;
  for (let i = 0; i < 5; i++) { // 迭代逼近
    const [gcjLng, gcjLat] = wgs84ToGcj02(wgsLng, wgsLat);
    wgsLng += lng - gcjLng;
    wgsLat += lat - gcjLat;
  }
  return [wgsLng, wgsLat];
}
```

### 9.4 前端推荐库

| 库名 | 功能 | 安装 |
|------|------|------|
| `coordtransform` | WGS84/GCJ-02/BD-09 互转 | `npm i coordtransform` |
| `gcoord` | 更全面，支持更多坐标系 | `npm i gcoord` |
| `@amap/amap-jsapi-loader` | 高德官方加载器 | 配合高德 SDK 使用 |

---

## 十、GIS Web 开发框架

### 10.1 四大框架对比

| 特性 | OpenLayers | Mapbox GL JS | Leaflet | Cesium |
|------|-----------|-------------|---------|--------|
| **定位** | 专业 GIS 2D | 高端可视化 2D/2.5D | 轻量 2D | 3D 地球 |
| **投影** | 支持 EPSG:4326、3857，通过 Proj4 扩展任意 EPSG | 仅 EPSG:3857（内部）+ 4326（输入） | 仅 EPSG:3857（插件扩展） | 仅 WGS84（3D 椭球体） |
| **矢量格式** | GeoJSON、KML、GML、WKT、TopoJSON | GeoJSON、MVT | GeoJSON、KML（插件） | GeoJSON、KML、CZML、3D Tiles |
| **地图服务** | WMS/WFS/WMTS/TMS/XYZ 全支持 | 自有 Vector Tiles + 栅格 | XYZ/TMS/WMS（插件） | 3D Tiles、Terrain、Imagery |
| **开源** | ✅ BSD | ❌ 需 token | ✅ BSD | ✅ Apache 2.0 |
| **学习曲线** | 陡 | 中等 | 平缓 | 陡 |
| **包大小** | ~400KB | ~800KB | ~40KB | ~4MB+ |

### 10.2 OpenLayers 投影与坐标处理

```javascript
import Map from 'ol/Map';
import View from 'ol/View';
import { fromLonLat, toLonLat, transform } from 'ol/proj';

// View 投影设置
const map = new Map({
  view: new View({
    projection: 'EPSG:3857',  // 默认
    center: fromLonLat([116.4074, 39.9042]),  // 经纬度 → Web Mercator
    zoom: 10
  })
});

// 坐标转换
const wmCoord = fromLonLat([116.4074, 39.9042]);  // 4326 → 3857
const lonLat = toLonLat(wmCoord);                  // 3857 → 4326

// 通用转换
const result = transform([116.4074, 39.9042], 'EPSG:4326', 'EPSG:3857');

// 自定义 EPSG（需 Proj4）
import proj4 from 'proj4';
import { register } from 'ol/proj/proj4';
proj4.defs('EPSG:4490', '+proj=longlat +ellps=GRS80 +no_defs');
register(proj4);
```

### 10.3 Mapbox GL JS 投影与坐标处理

```javascript
// Mapbox 内部使用 EPSG:3857 渲染，输入接受 EPSG:4326 经纬度
const map = new mapboxgl.Map({
  style: 'mapbox://styles/mapbox/streets-v12',
  center: [116.4074, 39.9042],  // [经度, 纬度]，4326
  zoom: 10
});

// 添加 GeoJSON 数据源（必须是 4326）
map.addSource('points', {
  type: 'geojson',
  data: {
    type: 'FeatureCollection',
    features: [...]
  }
});

// Mapbox 从 v2.0+ 支持投影设置
map.setProjection({ name: 'mercator' });  // 或 'globe', 'naturalEarth' 等
```

### 10.4 Cesium 坐标系统

```javascript
// Cesium 使用 WGS84 椭球体，三种主要坐标表示：
// 1. Cartographic（经纬度弧度 + 高度）
// 2. Cartesian3（地心直角坐标 XYZ）
// 3. 屏幕坐标（像素 XY）

// 经纬度 → Cartesian3
const position = Cesium.Cartesian3.fromDegrees(116.4074, 39.9042, 0);

// Cartesian3 → 经纬度
const cartographic = Cesium.Cartographic.fromCartesian(position);
const lon = Cesium.Math.toDegrees(cartographic.longitude);
const lat = Cesium.Math.toDegrees(cartographic.latitude);

// 加载 3D Tiles
const tileset = viewer.scene.primitives.add(
  new Cesium.Cesium3DTileset({ url: 'tileset.json' })
);
```

### 10.5 前端开发实战建议

| 场景 | 推荐方案 |
|------|---------|
| 国内地图 + 需要合规 | 高德 JS API / 天地图 API |
| 专业 GIS + 多种服务 | OpenLayers |
| 高端可视化 + 自定义样式 | Mapbox GL JS |
| 3D 地球 + 三维场景 | Cesium |
| 简单展示 + 快速上手 | Leaflet |
| 加载 CGCS2000 数据 | OpenLayers + Proj4 注册 EPSG:4490 |
| 国内坐标偏移 | `gcoord` 或 `coordtransform` 库 |

---

## 关键概念速查表

| 概念 | 一句话总结 |
|------|-----------|
| 地理坐标系 | 用经纬度描述位置，基于椭球+基准面 |
| 投影坐标系 | 用米描述位置，地理坐标系+投影方法 |
| Web Mercator | Web 地图事实标准，等角但面积严重变形 |
| EPSG:4326 | WGS84 经纬度，GPS 原始坐标 |
| EPSG:3857 | Web Mercator，Web 地图渲染用 |
| EPSG:4490 | CGCS2000，中国标准，≈ WGS84 |
| GCJ-02 | 中国法定偏移，高德/腾讯使用 |
| BD-09 | 百度二次偏移 |
| XYZ 瓦片 | Y 从上往下，最简单 |
| TMS 瓦片 | Y 从下往上 |
| WMTS | OGC 标准瓦片，企业级 |
| 矢量瓦片 | MVT 格式，客户端渲染，体积小 |
| GeoJSON | Web GIS 数据交换标准，[经度,纬度] 顺序 |
| 大地基准面 | 椭球体 + 定位 = 坐标系的基础 |
