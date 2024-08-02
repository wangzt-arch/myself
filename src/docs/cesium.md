# 常用方法
## cesium坐标转换
~~~js
//经纬度转换Cartesian3坐标
let pos = Cesium.Cartesian3.fromDegrees(61.296382224724795,35.628536117000692); 

//Cartesian3转经纬度坐标
//Cartographic坐标
let carto  = Cesium.Ellipsoid.WGS84.cartesianToCartographic(pos);
//经纬度 
let lon = Cesium.Math.toDegrees(carto.longitude); 
let lat = Cesium.Math.toDegrees(carto.latitude); 
~~~

## 屏幕坐标获取经纬度坐标
~~~js
// 方法1. 获取椭球体表面的经纬度坐标，
let handler = new Cesium.ScreenSpaceEventHandler(scene.canvas);
handler.setInputAction(function(evt) {
    let cartesian = viewer.camera.pickEllipsoid(evt.position,viewer.scene.globe.ellipsoid);
    let cartographic = Cesium.Cartographic.fromCartesian(cartesian);
    let lng = Cesium.Math.toDegrees(cartographic.longitude);//经度值
    let lat = Cesium.Math.toDegrees(cartographic.latitude);//纬度值
    let mapPosition = {x:lng,y:lat,z:cartographic.height};//cartographic.height的值始终为零。
}, Cesium.ScreenSpaceEventType.LEFT_CLICK);

//方法2.  获取地形表面的经纬度高程坐标
let handler = new Cesium.ScreenSpaceEventHandler(scene.canvas);
handler.setInputAction(function(evt) {
    let ray = viewer.camera.getPickRay(evt.position);
    let cartesian = viewer.scene.globe.pick(ray,viewer.scene);
    let cartographic = Cesium.Cartographic.fromCartesian(cartesian);
    let lng = Cesium.Math.toDegrees(cartographic.longitude);//经度值
    let lat = Cesium.Math.toDegrees(cartographic.latitude);//纬度值
    let mapPosition={x:lng,y:lat,z:cartographic.height};//cartographic.height的值为地形高度。
}, Cesium.ScreenSpaceEventType.LEFT_CLICK);

//方法3.  获取模型或者其他要素
let handler = new Cesium.ScreenSpaceEventHandler(scene.canvas);
handler.setInputAction(function(evt) {
    let cartesian = viewer.scene.pickPosition(evt.position);
    let cartographic = Cesium.Cartographic.fromCartesian(cartesian);
    let lng = Cesium.Math.toDegrees(cartographic.longitude);//经度值
    let lat = Cesium.Math.toDegrees(cartographic.latitude);//纬度值
    let mapPosition={x:lng,y:lat,z:cartographic.height};//
}, Cesium.ScreenSpaceEventType.LEFT_CLICK);

~~~

## 世界坐标转经纬度
~~~js
/**
 * 世界坐标转经纬度
 * @param {Cesium.Cartesian3 } cartesian 世界坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns { Array } 经纬度坐标s
 */

const cartesianToLnglat = function (cartesian, viewer) {
  if (!cartesian) return [];
  var lnglat = Cesium.Cartographic.fromCartesian(cartesian);
  var lat = Cesium.Math.toDegrees(lnglat.latitude);
  var lng = Cesium.Math.toDegrees(lnglat.longitude);
  var hei = lnglat.height;
  return [lng, lat, hei];
};
~~~

## 视角定位方法
~~~js
const flyTo = function (opt, viewer) {
  if (!viewer) {
    console.log('缺少viewer对象');
    return;
  }

  opt = opt || {};
  var center = opt.center;

  if (!center) {
    console.log("缺少定位坐标！");
    return;
  }

  delete opt.center;
  var options = {
    duration: opt.duration || 0,
    offset: new Cesium.HeadingPitchRange(Cesium.Math.toRadians(opt.heading || 0), Cesium.Math.toRadians(opt.pitch || -60), opt.range || 10000)
  };
  delete opt.heading;
  delete opt.pitch;
  delete opt.range;
  options = Object.assign(options, opt);

  if (center instanceof Cesium.Cartesian3) {
    viewer.camera.flyToBoundingSphere(new Cesium.BoundingSphere(center), options);
  }

  if (center instanceof Array) {
    var boundingSphere = new Cesium.BoundingSphere(Cesium.Cartesian3.fromDegrees(center[0], center[1], center[2]));
    viewer.camera.flyToBoundingSphere(boundingSphere, options);
  }
};
~~~

## 获取当相机姿态
~~~js
/**
 * 获取当相机姿态
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Object} cameraView 当前相机姿态
 */


const getCameraView = function (viewer) {
  viewer = viewer || window.viewer;

  if (!viewer) {
    console.log('缺少viewer对象');
    return;
  }

  var camera = viewer.camera;
  var position = camera.position;
  var heading = camera.heading;
  var pitch = camera.pitch;
  var roll = camera.roll;
  var lnglat = Cesium.Cartographic.fromCartesian(position);
  var cameraV = {
    "x": Cesium.Math.toDegrees(lnglat.longitude),
    "y": Cesium.Math.toDegrees(lnglat.latitude),
    "z": lnglat.height,
    "heading": Cesium.Math.toDegrees(heading),
    "pitch": Cesium.Math.toDegrees(pitch),
    "roll": Cesium.Math.toDegrees(roll)
  };
  return cameraV;
};

~~~

## 设置相机姿态

~~~js
/**
 * 设置相机姿态 一般和getCameraView搭配使用
 * @param {Object} cameraView 相机姿态参数
 * @param {Number} cameraView.duration 定位所需时间
 * @param {Cesium.Viewer} viewer 当前viewer对象
 */

const setCameraView = function (obj, viewer) {
  viewer = viewer || window.viewer;

  if (!viewer) {
    console.log('缺少viewer对象');
    return;
  }

  if (!obj) return;
  var position = obj.destination || Cesium.Cartesian3.fromDegrees(obj.x, obj.y, obj.z); // 兼容cartesian3和xyz

  viewer.camera.flyTo({
    destination: position,
    orientation: {
      heading: Cesium.Math.toRadians(obj.heading || 0),
      pitch: Cesium.Math.toRadians(obj.pitch || 0),
      roll: Cesium.Math.toRadians(obj.roll || 0)
    },
    duration: obj.duration === undefined ? 3 : obj.duration,
    complete: obj.complete
  });
};
~~~

## 获取视图中心点坐标

~~~js
const getViewCenter = function (viewer) {
  if (!viewer) return;
  var rectangle = viewer.camera.computeViewRectangle();
  var west = rectangle.west / Math.PI * 180;
  var north = rectangle.north / Math.PI * 180;
  var east = rectangle.east / Math.PI * 180;
  var south = rectangle.south / Math.PI * 180;
  return [(east + west) / 2, (north + south) / 2];
};
~~~

## 由四元数计算偏转角（heading）、仰俯角（pitch）、翻滚角（roll）
~~~js
/**
 * 由四元数计算偏转角（heading）、仰俯角（pitch）、翻滚角（roll）
 * @param {Cesium.Cartesian3} position 中心点坐标
 * @param {Cesium.Quaternion} orientation 四元数
 * @param {Boolean} toDegrees true，转化为度 / false，转为弧度
 * @returns {Object} hpr 姿态参数
 */


const oreatationToHpr = function (position, orientation, toDegrees) {
  if (!position || !orientation) return;
  var matrix3Scratch = new Cesium.Matrix3();
  var mtx3 = Cesium.Matrix3.fromQuaternion(orientation, matrix3Scratch);
  var mtx4 = Cesium.Matrix4.fromRotationTranslation(mtx3, position, new Cesium.Matrix4());
  var hpr = Cesium.Transforms.fixedFrameToHeadingPitchRoll(mtx4, Cesium.Ellipsoid.WGS84, Cesium.Transforms.eastNorthUpToFixedFrame, new Cesium.HeadingPitchRoll());
  var heading = hpr.heading,
      pitch = hpr.pitch,
      roll = hpr.roll;

  if (toDegrees) {
    // 是否转化为度
    heading = Cesium.Math.toDegrees(heading);
    pitch = Cesium.Math.toDegrees(pitch);
    roll = Cesium.Math.toDegrees(roll);
  }

  return {
    heading: heading,
    pitch: pitch,
    roll: roll
  };
}; 
~~~

## 坐标转换
~~~js
var PI = 3.1415926535897932384626;
var a$1 = 6378245.0;
var ee = 0.00669342162296594323;

function transformWD(lng, lat) {
  var ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * Math.sqrt(Math.abs(lng));
  ret += (20.0 * Math.sin(6.0 * lng * PI) + 20.0 * Math.sin(2.0 * lng * PI)) * 2.0 / 3.0;
  ret += (20.0 * Math.sin(lat * PI) + 40.0 * Math.sin(lat / 3.0 * PI)) * 2.0 / 3.0;
  ret += (160.0 * Math.sin(lat / 12.0 * PI) + 320 * Math.sin(lat * PI / 30.0)) * 2.0 / 3.0;
  return ret;
}

function transformJD(lng, lat) {
  var ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * Math.sqrt(Math.abs(lng));
  ret += (20.0 * Math.sin(6.0 * lng * PI) + 20.0 * Math.sin(2.0 * lng * PI)) * 2.0 / 3.0;
  ret += (20.0 * Math.sin(lng * PI) + 40.0 * Math.sin(lng / 3.0 * PI)) * 2.0 / 3.0;
  ret += (150.0 * Math.sin(lng / 12.0 * PI) + 300.0 * Math.sin(lng / 30.0 * PI)) * 2.0 / 3.0;
  return ret;
}

const wgs2gcj = function (arrdata) {
  var lng = Number(arrdata[0]);
  var lat = Number(arrdata[1]);
  var dlat = transformWD(lng - 105.0, lat - 35.0);
  var dlng = transformJD(lng - 105.0, lat - 35.0);
  var radlat = lat / 180.0 * PI;
  var magic = Math.sin(radlat);
  magic = 1 - ee * magic * magic;
  var sqrtmagic = Math.sqrt(magic);
  dlat = dlat * 180.0 / (a$1 * (1 - ee) / (magic * sqrtmagic) * PI);
  dlng = dlng * 180.0 / (a$1 / sqrtmagic * Math.cos(radlat) * PI);
  var mglat = lat + dlat;
  var mglng = lng + dlng;
  mglng = Number(mglng.toFixed(6));
  mglat = Number(mglat.toFixed(6));
  return [mglng, mglat];
};

const gcj2wgs = function (arrdata) {
  var lng = Number(arrdata[0]);
  var lat = Number(arrdata[1]);
  var dlat = transformWD(lng - 105.0, lat - 35.0);
  var dlng = transformJD(lng - 105.0, lat - 35.0);
  var radlat = lat / 180.0 * PI;
  var magic = Math.sin(radlat);
  magic = 1 - ee * magic * magic;
  var sqrtmagic = Math.sqrt(magic);
  dlat = dlat * 180.0 / (a$1 * (1 - ee) / (magic * sqrtmagic) * PI);
  dlng = dlng * 180.0 / (a$1 / sqrtmagic * Math.cos(radlat) * PI);
  var mglat = lat + dlat;
  var mglng = lng + dlng;
  var jd = lng * 2 - mglng;
  var wd = lat * 2 - mglat;
  jd = Number(jd.toFixed(6));
  wd = Number(wd.toFixed(6));
  return [jd, wd];
};
~~~

## 由两点计算和地形以及模型的交点
~~~js
/**
 * 由两点计算和地形以及模型的交点
 * @param {Object} obj 坐标参数
 * @param {Cesium.Cartesian3 } obj.startPoint 起点坐标
 * @param {Cesium.Cartesian3 } obj.endPoint 终点坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Cesium.Cartesian3 } 交点坐标
 */


const getIntersectPosition = function (obj, viewer) {
  if (!viewer) {
    console.log('缺少viewer对象');
    return;
  }

  var p1 = obj.startPoint;
  var p2 = obj.endPoint;

  if (!p1 || !p2) {
    console.log("缺少坐标！");
    return;
  }

  var direction = Cesium.Cartesian3.subtract(p2.clone(), p1.clone(), new Cesium.Cartesian3());
  direction = Cesium.Cartesian3.normalize(direction, new Cesium.Cartesian3());
  var ray = new Cesium.Ray(p1.clone(), direction.clone());
  var pick = viewer.scene.pickFromRay(ray);
  if (!pick) return null;
  return pick.position;
};
~~~

## 由中心点、圆上某点以及角度 计算圆上其它点坐标 
~~~js
/**
 * 由中心点、圆上某点以及角度 计算圆上其它点坐标 
 * @param {Cesium.Cartesian3 } center 圆的中心点 
 * @param {Cesium.Cartesian3 } aimP 圆上某点
 * @param {Number} [angle] 间隔角度，默认为60° 
 * @returns {Cesium.Cartesian3[]} 圆上点坐标数组
 */
const getCirclePointsByAngle = function (center, aimP, angle) {
  var dis = Cesium.Cartesian3.distance(center.clone(), aimP.clone());
  var circlePositions = [];
  angle = angle || 60;

  for (var i = 0; i < 360; i += angle) {
    // 旋转矩阵
    var hpr = new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(i), Cesium.Math.toRadians(0), Cesium.Math.toRadians(0));
    var mtx4 = Cesium.Transforms.headingPitchRollToFixedFrame(center.clone(), hpr);
    var mtx3 = Cesium.Matrix4.getMatrix3(mtx4, new Cesium.Matrix3());
    var newPosition = Cesium.Matrix3.multiplyByVector(mtx3, aimP.clone(), new Cesium.Cartesian3());
    var dir = Cesium.Cartesian3.subtract(newPosition.clone(), center.clone(), new Cesium.Cartesian3());
    dir = Cesium.Cartesian3.normalize(dir, new Cesium.Cartesian3());
    dir = Cesium.Cartesian3.multiplyByScalar(dir, dis, new Cesium.Cartesian3());
    newPosition = Cesium.Cartesian3.add(center.clone(), dir.clone(), new Cesium.Cartesian3());
    var ctgc = Cesium.Cartographic.fromCartesian(newPosition.clone());
    circlePositions.push(ctgc);
  }

  circlePositions.unshift();
  return circlePositions;
};
~~~

## 由中心点、半径以及角度 计算圆上其它点坐标 
~~~js
/**
 * 由中心点、半径以及角度 计算圆上其它点坐标 
 * @param {Cesium.Cartesian3 } center 圆的中心点 
 * @param {Number} radius 半径长度
 * @param {Number} [angle] 间隔角度，默认为60° 
 * @returns {Cesium.Cartesian3[]} 圆上点坐标数组
 */


const getCirclePointsByRadius = function (opt) {
  var _ref = opt || {},
      center = _ref.center,
      radius = _ref.radius,
      angle = _ref.angle;

  if (!center || !radius) return;
  angle = angle || 60;
  var positions = []; // 局部坐标系到世界坐标系的矩阵

  var mtx4 = Cesium.Transforms.eastNorthUpToFixedFrame(center.clone()); // 世界到局部

  var mtx4_inverse = Cesium.Matrix4.inverse(mtx4, new Cesium.Matrix4());
  var local_center = Cesium.Matrix4.multiplyByPoint(mtx4_inverse, center.clone(), new Cesium.Cartesian3());
  var rposition = Cesium.Cartesian3.add(local_center, new Cesium.Cartesian3(radius, 0, 0), new Cesium.Cartesian3());

  for (var i = 0; i <= 360; i += angle) {
    var radians = Cesium.Math.toRadians(i);
    var mtx3 = Cesium.Matrix3.fromRotationZ(radians);
    var newPosition = Cesium.Matrix3.multiplyByVector(mtx3, rposition.clone(), new Cesium.Cartesian3());
    newPosition = Cesium.Matrix4.multiplyByPoint(mtx4, newPosition.clone(), new Cesium.Cartesian3());
    positions.push(newPosition);
  }

  return positions;
};
~~~

## 计算两点连线夹角
~~~js
/**
 * 计算两点连线夹角
 * @param {Cartographic} p1 
 * @param {Cartographic} p2 
 * @returns {Number} bearing 角度
 */


const computeAngle = function (p1, p2) {
  var lng_a = p1.longitude;
  var lat_a = p1.latitude;
  var lng_b = p2.longitude;
  var lat_b = p2.latitude;
  var y = Math.sin(lng_b - lng_a) * Math.cos(lat_b);
  var x = Math.cos(lat_a) * Math.sin(lat_b) - Math.sin(lat_a) * Math.cos(lat_b) * Math.cos(lng_b - lng_a);
  var bearing = Math.atan2(y, x);
  bearing = bearing * 180.0 / Math.PI;

  if (bearing < -180) {
    bearing = bearing + 360;
  }

  return bearing;
};
~~~

## 获取地形高度
~~~js
/**
 * 获取地形高度
 * @param {Cesium.Cartesian3 } position 当前点坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Number} height，当前坐标点的地形高度
 */


const getTerrainHeight = function (position, viewer) {
  if (!viewer) {
    console.log('缺少viewer对象');
    return;
  }

  if (!position || !viewer) return;
  return viewer.scene.globe.getHeight(Cesium.Cartographic.fromCartesian(position));
};
~~~

## 获取高度，包含地形和模型上的高度
~~~js
/**
 * 获取高度，包含地形和模型上的高度
 * @param {Cesium.Cartesian3 } position 当前点坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Number} height，当前坐标点的高度
 */


const get3dtilesHeight = function (position, viewer) {
  if (!viewer) {
    console.log('缺少viewer对象');
    return;
  }

  if (!position || !viewer) return;
  return viewer.scene.sampleHeight(Cesium.Cartographic.fromCartesian(position));
};
~~~

## 计算当前三角形面积
~~~js
/**
 * 计算当前三角形面积
 * @param {Cesium.Cartesian3 } pos1 当前点坐标1
 * @param {Cesium.Cartesian3 } pos2 当前点坐标2
 * @param {Cesium.Cartesian3 } pos3 当前点坐标3
 * @returns {Number} area，面积
 */


const computeAreaOfTriangle = function (pos1, pos2, pos3) {
  if (!pos1 || !pos2 || !pos3) {
    console.log("传入坐标有误！");
    return 0;
  }

  var a = Cesium.Cartesian3.distance(pos1, pos2);
  var b = Cesium.Cartesian3.distance(pos2, pos3);
  var c = Cesium.Cartesian3.distance(pos3, pos1);
  var S = (a + b + c) / 2;
  return Math.sqrt(S * (S - a) * (S - b) * (S - c));
};
~~~
