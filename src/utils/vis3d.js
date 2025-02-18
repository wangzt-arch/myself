import { area, booleanPointInPolygon, center, featureCollection, point, polygon } from '@turf/turf';
// import { map, tileLayer, latLng, latLngBounds, rectangle } from 'leaflet';
// import 'leaflet/dist/leaflet.css';
import axios from 'axios';
import { extendComponentModel, extendComponentView, graphic as graphic$1, init, matrix, registerAction, registerCoordinateSystem } from 'echarts';
import * as mapv$1 from 'mapv';
import { DataSet, baiduMapLayer as baiduMapLayer$1 } from 'mapv';

/**
 * 三维基础方法
 * @example util.getCameraView(viewer);
 * @exports util
 * @alias util
 */

var util$1 = {};
/**
 * 世界坐标转经纬度
 * @param {Cesium.Cartesian3 } cartesian 世界坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns { Array } 经纬度坐标s
 */

util$1.cartesianToLnglat = function (cartesian, viewer) {
  if (!cartesian) return [];
  var lnglat = Cesium.Cartographic.fromCartesian(cartesian);
  var lat = Cesium.Math.toDegrees(lnglat.latitude);
  var lng = Cesium.Math.toDegrees(lnglat.longitude);
  var hei = lnglat.height;
  return [lng, lat, hei];
};

util$1.c2jwd = function (cartesian, viewer) {
  return util$1.cartesianToLnglat(cartesian, viewer);
};
/**
 * 世界坐标数组转经纬度数组
 * @param {Cesium.Cartesian3[]} cartesians 世界坐标数组
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns { Array } 经纬度坐标数组
 */


util$1.cartesiansToLnglats = function (cartesians, viewer) {
  if (!cartesians || cartesians.length < 1) return;
  viewer = viewer || window.viewer;

  if (!viewer) {
    console.log('util.cartesiansToLnglats方法缺少viewer对象');
    return;
  }

  var arr = [];

  for (var i = 0; i < cartesians.length; i++) {
    arr.push(util$1.cartesianToLnglat(cartesians[i], viewer));
  }

  return arr;
};

util$1.cs2jwds = function (cartesians, viewer) {
  return util$1.cartesiansToLnglats(cartesians, viewer);
};
/**
 * 经纬度坐标数组转世界坐标数组
 * @param {Array[]} lnglats 经纬度坐标数组
 * @returns {Cesium.Cartesian3[]} cartesians 世界坐标数组
 * @example util.lnglatsToCartesians([[117,40],[118.41]])
 */


util$1.lnglatsToCartesians = function (lnglats) {
  if (!lnglats || lnglats.length < 1) return;
  var arr = [];

  for (var i = 0; i < lnglats.length; i++) {
    var c3 = Cesium.Cartesian3.fromDegrees(Number(lnglats[i][0]), Number(lnglats[i][1]), Number(lnglats[i][2] || 0));
    arr.push(c3);
  }

  return arr;
};

util$1.jwds2cs = function (cartesians, viewer) {
  return util$1.lnglatsToCartesians(cartesians, viewer);
};
/**
 * 视角定位方法
 * @param {Object} opt 定位参数
 * @param {Cartesian3|Array} opt.center 当前定位中心点
 * @param {Number} opt.heading 当前定位偏转角度 默认为0 
 * @param {Number} opt.pitch 当前定位仰俯角 默认为-60
 * @param {Number} opt.range 当前定位距离 默认为1000米
 * @param {Cesium.Viewer} viewer 当前viewer对象
 */


util$1.flyTo = function (opt, viewer) {
  if (!viewer) {
    console.log('util.flyTo缺少viewer对象');
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
/**
 * 获取当相机姿态
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Object} cameraView 当前相机姿态
 */


util$1.getCameraView = function (viewer) {
  viewer = viewer || window.viewer;

  if (!viewer) {
    console.log('util.getCameraView缺少viewer对象');
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
/**
 * 设置相机姿态 一般和getCameraView搭配使用
 * @param {Object} cameraView 相机姿态参数
 * @param {Number} cameraView.duration 定位所需时间
 * @param {Cesium.Viewer} viewer 当前viewer对象
 */


util$1.setCameraView = function (obj, viewer) {
  viewer = viewer || window.viewer;

  if (!viewer) {
    console.log('util.setCameraView缺少viewer对象');
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
/**
 * 获取视图中心点坐标
 */


util$1.getViewCenter = function (viewer) {
  if (!viewer) return;
  var rectangle = viewer.camera.computeViewRectangle();
  var west = rectangle.west / Math.PI * 180;
  var north = rectangle.north / Math.PI * 180;
  var east = rectangle.east / Math.PI * 180;
  var south = rectangle.south / Math.PI * 180;
  return [(east + west) / 2, (north + south) / 2];
};
/**
 * 由四元数计算偏转角（heading）、仰俯角（pitch）、翻滚角（roll）
 * @param {Cesium.Cartesian3} position 中心点坐标
 * @param {Cesium.Quaternion} orientation 四元数
 * @param {Boolean} toDegrees true，转化为度 / false，转为弧度
 * @returns {Object} hpr 姿态参数
 */


util$1.oreatationToHpr = function (position, orientation, toDegrees) {
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
}; // ================================== 坐标转换 ==================================
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

util$1.wgs2gcj = function (arrdata) {
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

util$1.gcj2wgs = function (arrdata) {
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
/**
 * 坐标插值方法
 * @param {Cesium.Cartesian3[]} positions 世界坐标数组
 * @param {Number} [granularity] 插值粒度，默认为0.00001，值越小，插值越多
 * @returns {Cesium.Cartesian3[]} newPositions 转换后世界坐标数组
 */


util$1.lerpPositions = function (positions, granularity) {
  if (!positions || positions.length == 0) return;
  var dis = 0;

  for (var i = 1; i < positions.length; i++) {
    dis += Cesium.Cartesian3.distance(positions[i], positions[i - 1]);
  }

  var surfacePositions = Cesium.PolylinePipeline.generateArc({
    //将线进行插值
    positions: positions,
    granularity: granularity || 0.000000001 * dis
  });
  if (!surfacePositions) return;
  var arr = [];

  for (var _i = 0; _i < surfacePositions.length; _i += 3) {
    var cartesian = Cesium.Cartesian3.unpack(surfacePositions, _i); //分组

    arr.push(cartesian);
  }

  return arr;
};
/**
 * 由两点计算和地形以及模型的交点
 * @param {Object} obj 坐标参数
 * @param {Cesium.Cartesian3 } obj.startPoint 起点坐标
 * @param {Cesium.Cartesian3 } obj.endPoint 终点坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Cesium.Cartesian3 } 交点坐标
 */


util$1.getIntersectPosition = function (obj, viewer) {
  if (!viewer) {
    console.log('util.getIntersectPosition缺少viewer对象');
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
/**
 * 由中心点、圆上某点以及角度 计算圆上其它点坐标 
 * @param {Cesium.Cartesian3 } center 圆的中心点 
 * @param {Cesium.Cartesian3 } aimP 圆上某点
 * @param {Number} [angle] 间隔角度，默认为60° 
 * @returns {Cesium.Cartesian3[]} 圆上点坐标数组
 */


util$1.getCirclePointsByAngle = function (center, aimP, angle) {
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
/**
 * 由中心点、半径以及角度 计算圆上其它点坐标 
 * @param {Cesium.Cartesian3 } center 圆的中心点 
 * @param {Number} radius 半径长度
 * @param {Number} [angle] 间隔角度，默认为60° 
 * @returns {Cesium.Cartesian3[]} 圆上点坐标数组
 */


util$1.getCirclePointsByRadius = function (opt) {
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
/**
 * 计算两点连线夹角
 * @param {Cartographic} p1 
 * @param {Cartographic} p2 
 * @returns {Number} bearing 角度
 */


util$1.computeAngle = function (p1, p2) {
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
/**
 * 修改当前世界坐标数组中坐标的高度
 * @param {Cesium.Cartesian3[]} positions 世界坐标数组 
 * @param {Number} h 坐标高度 
 * @returns {Cesium.Cartesian3[]} newPoisitions 修改高度后的世界坐标数组 
 */


util$1.updatePositionsHeight = function (pois, h) {
  if (!pois || h == undefined) return;
  var newPois = [];

  for (var i = 0; i < pois.length; i++) {
    var c3 = pois[i];
    var ct = util$1.cartesianToLnglat(c3);
    var newC3 = Cesium.Cartesian3.fromDegrees(ct[0], ct[1], h);
    newPois.push(newC3);
  }

  return newPois;
};
/**
 * 对世界坐标数组进行面状插值
 * @param {Cesium.Cartesian3[]} positions 世界坐标数组
 * @param {Boolean} isOn3dtiles 是否在模型上 
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Object} data 返回值，包含uniformArr（对象数组，每个对象中包含当前片元面积及高度），minHeight（当前范围内最小高度），maxHeight（当前范围内最大高度）
 * 
 */


util$1.computeUniforms = function (positions, isOn3dtiles, viewer) {
  if (!viewer) {
    console.log('util.computeUniforms缺少viewer对象');
    return;
  }

  var area = util$1.computeArea(positions, viewer) / 1000;
  if (!positions) return;
  var polygonGeometry = new Cesium.PolygonGeometry.fromPositions({
    positions: positions,
    vertexFormat: Cesium.PerInstanceColorAppearance.FLAT_VERTEX_FORMAT,
    granularity: Math.PI / Math.pow(2, 11) / 1000 * (area / 10)
  });
  var geom = new Cesium.PolygonGeometry.createGeometry(polygonGeometry);
  var indices = geom.indices;
  var attrPosition = geom.attributes.position;
  var data = {};
  data.uniformArr = [];
  data.minHeight = Number.MAX_VALUE;
  data.maxHeight = Number.MIN_VALUE;

  for (var index = 0; index < indices.length; index = index + 3) {
    var obj = {};
    var first = indices[index];
    var second = indices[index + 1];
    var third = indices[index + 2];
    var cartesian1 = new Cesium.Cartesian3(attrPosition.values[first * 3], geom.attributes.position.values[first * 3 + 1], attrPosition.values[first * 3 + 2]);
    var h1 = void 0;

    if (!isOn3dtiles) {
      h1 = util$1.getTerrainHeight(cartesian1, viewer);
    } else {
      h1 = util$1.get3dtilesHeight(cartesian1, viewer);
    }

    var cartesian2 = new Cesium.Cartesian3(attrPosition.values[second * 3], geom.attributes.position.values[second * 3 + 1], attrPosition.values[second * 3 + 2]);
    var h2 = void 0;

    if (!isOn3dtiles) {
      h2 = util$1.getTerrainHeight(cartesian2, viewer);
    } else {
      h2 = util$1.get3dtilesHeight(cartesian2, viewer);
    }

    var cartesian3 = new Cesium.Cartesian3(geom.attributes.position.values[third * 3], geom.attributes.position.values[third * 3 + 1], attrPosition.values[third * 3 + 2]);
    var h3 = void 0;

    if (!isOn3dtiles) {
      h3 = util$1.getTerrainHeight(cartesian3, viewer);
    } else {
      h3 = util$1.get3dtilesHeight(cartesian3, viewer);
    }

    obj.height = (h1 + h2 + h3) / 3;

    if (data.minHeight > obj.height) {
      data.minHeight = obj.height;
    }

    if (data.maxHeight < obj.height) {
      data.maxHeight = obj.height;
    }

    obj.area = util$1.computeAreaOfTriangle(cartesian1, cartesian2, cartesian3);
    data.uniformArr.push(obj);
  }

  return data;
};
/**
 * 计算面积
 * @param {Cesium.Cartesian3[]} positions 世界坐标数组
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Number} area，面积
 */


util$1.computeArea = function (positions, viewer) {
  if (!viewer) {
    console.log('util.computeArea缺少viewer对象');
    return;
  }

  positions = positions.concat([positions[0]]);
  var lnglats = util$1.cartesiansToLnglats(positions, viewer);
  var polygon$1 = polygon([lnglats]);
  var area$1 = area(polygon$1);
  return area$1;
};
/**
 * 判断点是否在面内
 * @param {Array} point 点的经纬度坐标
 * @param {Array[]} lnglats 面经纬度坐标
 */


util$1.isPointInPolygon = function (point$1, lnglats) {
  var pt = point(point$1);
  lnglats[0] = lnglats[0].concat([lnglats[0][0]]);
  var poly = polygon([lnglats]);
  return booleanPointInPolygon(pt, poly);
};
/**
 * 获取地形高度
 * @param {Cesium.Cartesian3 } position 当前点坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Number} height，当前坐标点的地形高度
 */


util$1.getTerrainHeight = function (position, viewer) {
  if (!viewer) {
    console.log('util.getTerrainHeight缺少viewer对象');
    return;
  }

  if (!position || !viewer) return;
  return viewer.scene.globe.getHeight(Cesium.Cartographic.fromCartesian(position));
};
/**
 * 获取高度，包含地形和模型上的高度
 * @param {Cesium.Cartesian3 } position 当前点坐标
 * @param {Cesium.Viewer} viewer 当前viewer对象
 * @returns {Number} height，当前坐标点的高度
 */


util$1.get3dtilesHeight = function (position, viewer) {
  if (!viewer) {
    console.log('util.get3dtilesHeight缺少viewer对象');
    return;
  }

  if (!position || !viewer) return;
  return viewer.scene.sampleHeight(Cesium.Cartographic.fromCartesian(position));
};
/**
 * 计算当前三角形面积
 * @param {Cesium.Cartesian3 } pos1 当前点坐标1
 * @param {Cesium.Cartesian3 } pos2 当前点坐标2
 * @param {Cesium.Cartesian3 } pos3 当前点坐标3
 * @returns {Number} area，面积
 */


util$1.computeAreaOfTriangle = function (pos1, pos2, pos3) {
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
/**
 * 计算地形坡度
 * @param {Cesium.Cartesian3 } center 
 * @param {Number} [radius] 坡度半径 
 * @param {Number} [angle] 插值角度 
 * @param {Cesium.Viewer} viewer 当前viewer对象 
 * @returns {Object} 返回坡度起始点坐标、终点坐标以及坡度值
 */


util$1.getSlopePosition = function (center, radius, angle, viewer) {
  if (!viewer) {
    console.log('util.getSlopePosition缺少viewer对象');
    return;
  }

  if (!viewer || !center) return;
  var positions = util$1.getCirclePointsByRadius({
    center: center,
    radius: radius || 10,
    angle: angle || 10
  }, viewer);
  var minH = Number.MAX_VALUE;
  var centerH = util$1.getTerrainHeight(center.clone(), viewer);
  if (!centerH) return;
  var step = -1;

  for (var i = 0; i < positions.length; i++) {
    var h = util$1.getTerrainHeight(positions[i], viewer);

    if (minH > h) {
      minH = h;
      step = i;
    }
  }

  var startP;
  var endP;

  if (minH < centerH) {
    startP = center.clone();
    endP = positions[step].clone();
  } else {
    startP = positions[step].clone();
    endP = center.clone();
  }

  var startCgtc = Cesium.Cartographic.fromCartesian(startP);
  var endCgtc = Cesium.Cartographic.fromCartesian(endP);
  startP = Cesium.Cartesian3.fromRadians(startCgtc.longitude, startCgtc.latitude, minH < centerH ? centerH : minH);
  endP = Cesium.Cartesian3.fromRadians(endCgtc.longitude, endCgtc.latitude, minH < centerH ? minH : centerH);
  var dis = Cesium.Cartesian3.distance(startP, endP);
  var height = Math.abs(centerH - minH);
  var sinAngle = height / dis;
  var slopeAngle = Math.acos(sinAngle);
  var slope = Cesium.Math.toDegrees(slopeAngle);
  return {
    startP: startP,
    endP: endP,
    slope: slope
  };
};

/**
 * 常用工具、和地图无关的方法
 * @exports tool
 * @alias tool
 */
var tool = {};
/**
 * 文本或json等下载方法
 * @param {String} fileName 文件名称，后缀需要加类型，如.txt / .json等
 * @param {String} datastr 文本字符串
 * @example tool.downloadFile("测试.json",JSON.stringify(data));
*/

tool.downloadFile = function (fileName, datastr) {
  var blob = new Blob([datastr]);

  _download(fileName, blob);
};
/**
 * 图片下载方法
 * @param {String} fileName 图片名称
 * @param {Canvas} canvas dom canvas对象
*/


tool.downloadImage = function (fileName, canvas) {
  var base64 = canvas.toDataURL("image/png");
  var blob = base64Img2Blob(base64);

  _download(fileName + '.png', blob);
};
/**
 * 树形结构转线性
 */


tool.tree2line = function (data, fieldName) {
  fieldName = fieldName || 'children';
  if (!data) return [];
  var arr = [];

  function recurse(item) {
    var itemCopy = JSON.parse(JSON.stringify(item));
    delete itemCopy[fieldName];

    if (item[fieldName] && item[fieldName].length > 0) {
      for (var i = 0; i < item[fieldName].length; i++) {
        var one = item[fieldName][i];
        one.parentAttr = itemCopy; // 保存父级属性

        recurse(one);
      }
    } else {
      arr.push(itemCopy);
    }
  }

  recurse(data);
  return arr;
};

function _download(fileName, blob) {
  var aLink = document.createElement('a');
  aLink.download = fileName;
  aLink.href = URL.createObjectURL(blob);
  document.body.appendChild(aLink);
  aLink.click();
  document.body.removeChild(aLink);
}

function base64Img2Blob(code) {
  var parts = code.split(';base64,');
  var contentType = parts[0].split(':')[1];
  var raw = window.atob(parts[1]);
  var rawLength = raw.length;
  var uInt8Array = new Uint8Array(rawLength);

  for (var i = 0; i < rawLength; ++i) {
    uInt8Array[i] = raw.charCodeAt(i);
  }

  return new Blob([uInt8Array], {
    type: contentType
  });
}

function _regeneratorRuntime() {
  /*! regenerator-runtime -- Copyright (c) 2014-present, Facebook, Inc. -- license (MIT): https://github.com/facebook/regenerator/blob/main/LICENSE */

  _regeneratorRuntime = function () {
    return exports;
  };

  var exports = {},
      Op = Object.prototype,
      hasOwn = Op.hasOwnProperty,
      $Symbol = "function" == typeof Symbol ? Symbol : {},
      iteratorSymbol = $Symbol.iterator || "@@iterator",
      asyncIteratorSymbol = $Symbol.asyncIterator || "@@asyncIterator",
      toStringTagSymbol = $Symbol.toStringTag || "@@toStringTag";

  function define(obj, key, value) {
    return Object.defineProperty(obj, key, {
      value: value,
      enumerable: !0,
      configurable: !0,
      writable: !0
    }), obj[key];
  }

  try {
    define({}, "");
  } catch (err) {
    define = function (obj, key, value) {
      return obj[key] = value;
    };
  }

  function wrap(innerFn, outerFn, self, tryLocsList) {
    var protoGenerator = outerFn && outerFn.prototype instanceof Generator ? outerFn : Generator,
        generator = Object.create(protoGenerator.prototype),
        context = new Context(tryLocsList || []);
    return generator._invoke = function (innerFn, self, context) {
      var state = "suspendedStart";
      return function (method, arg) {
        if ("executing" === state) throw new Error("Generator is already running");

        if ("completed" === state) {
          if ("throw" === method) throw arg;
          return doneResult();
        }

        for (context.method = method, context.arg = arg;;) {
          var delegate = context.delegate;

          if (delegate) {
            var delegateResult = maybeInvokeDelegate(delegate, context);

            if (delegateResult) {
              if (delegateResult === ContinueSentinel) continue;
              return delegateResult;
            }
          }

          if ("next" === context.method) context.sent = context._sent = context.arg;else if ("throw" === context.method) {
            if ("suspendedStart" === state) throw state = "completed", context.arg;
            context.dispatchException(context.arg);
          } else "return" === context.method && context.abrupt("return", context.arg);
          state = "executing";
          var record = tryCatch(innerFn, self, context);

          if ("normal" === record.type) {
            if (state = context.done ? "completed" : "suspendedYield", record.arg === ContinueSentinel) continue;
            return {
              value: record.arg,
              done: context.done
            };
          }

          "throw" === record.type && (state = "completed", context.method = "throw", context.arg = record.arg);
        }
      };
    }(innerFn, self, context), generator;
  }

  function tryCatch(fn, obj, arg) {
    try {
      return {
        type: "normal",
        arg: fn.call(obj, arg)
      };
    } catch (err) {
      return {
        type: "throw",
        arg: err
      };
    }
  }

  exports.wrap = wrap;
  var ContinueSentinel = {};

  function Generator() {}

  function GeneratorFunction() {}

  function GeneratorFunctionPrototype() {}

  var IteratorPrototype = {};
  define(IteratorPrototype, iteratorSymbol, function () {
    return this;
  });
  var getProto = Object.getPrototypeOf,
      NativeIteratorPrototype = getProto && getProto(getProto(values([])));
  NativeIteratorPrototype && NativeIteratorPrototype !== Op && hasOwn.call(NativeIteratorPrototype, iteratorSymbol) && (IteratorPrototype = NativeIteratorPrototype);
  var Gp = GeneratorFunctionPrototype.prototype = Generator.prototype = Object.create(IteratorPrototype);

  function defineIteratorMethods(prototype) {
    ["next", "throw", "return"].forEach(function (method) {
      define(prototype, method, function (arg) {
        return this._invoke(method, arg);
      });
    });
  }

  function AsyncIterator(generator, PromiseImpl) {
    function invoke(method, arg, resolve, reject) {
      var record = tryCatch(generator[method], generator, arg);

      if ("throw" !== record.type) {
        var result = record.arg,
            value = result.value;
        return value && "object" == typeof value && hasOwn.call(value, "__await") ? PromiseImpl.resolve(value.__await).then(function (value) {
          invoke("next", value, resolve, reject);
        }, function (err) {
          invoke("throw", err, resolve, reject);
        }) : PromiseImpl.resolve(value).then(function (unwrapped) {
          result.value = unwrapped, resolve(result);
        }, function (error) {
          return invoke("throw", error, resolve, reject);
        });
      }

      reject(record.arg);
    }

    var previousPromise;

    this._invoke = function (method, arg) {
      function callInvokeWithMethodAndArg() {
        return new PromiseImpl(function (resolve, reject) {
          invoke(method, arg, resolve, reject);
        });
      }

      return previousPromise = previousPromise ? previousPromise.then(callInvokeWithMethodAndArg, callInvokeWithMethodAndArg) : callInvokeWithMethodAndArg();
    };
  }

  function maybeInvokeDelegate(delegate, context) {
    var method = delegate.iterator[context.method];

    if (undefined === method) {
      if (context.delegate = null, "throw" === context.method) {
        if (delegate.iterator.return && (context.method = "return", context.arg = undefined, maybeInvokeDelegate(delegate, context), "throw" === context.method)) return ContinueSentinel;
        context.method = "throw", context.arg = new TypeError("The iterator does not provide a 'throw' method");
      }

      return ContinueSentinel;
    }

    var record = tryCatch(method, delegate.iterator, context.arg);
    if ("throw" === record.type) return context.method = "throw", context.arg = record.arg, context.delegate = null, ContinueSentinel;
    var info = record.arg;
    return info ? info.done ? (context[delegate.resultName] = info.value, context.next = delegate.nextLoc, "return" !== context.method && (context.method = "next", context.arg = undefined), context.delegate = null, ContinueSentinel) : info : (context.method = "throw", context.arg = new TypeError("iterator result is not an object"), context.delegate = null, ContinueSentinel);
  }

  function pushTryEntry(locs) {
    var entry = {
      tryLoc: locs[0]
    };
    1 in locs && (entry.catchLoc = locs[1]), 2 in locs && (entry.finallyLoc = locs[2], entry.afterLoc = locs[3]), this.tryEntries.push(entry);
  }

  function resetTryEntry(entry) {
    var record = entry.completion || {};
    record.type = "normal", delete record.arg, entry.completion = record;
  }

  function Context(tryLocsList) {
    this.tryEntries = [{
      tryLoc: "root"
    }], tryLocsList.forEach(pushTryEntry, this), this.reset(!0);
  }

  function values(iterable) {
    if (iterable) {
      var iteratorMethod = iterable[iteratorSymbol];
      if (iteratorMethod) return iteratorMethod.call(iterable);
      if ("function" == typeof iterable.next) return iterable;

      if (!isNaN(iterable.length)) {
        var i = -1,
            next = function next() {
          for (; ++i < iterable.length;) if (hasOwn.call(iterable, i)) return next.value = iterable[i], next.done = !1, next;

          return next.value = undefined, next.done = !0, next;
        };

        return next.next = next;
      }
    }

    return {
      next: doneResult
    };
  }

  function doneResult() {
    return {
      value: undefined,
      done: !0
    };
  }

  return GeneratorFunction.prototype = GeneratorFunctionPrototype, define(Gp, "constructor", GeneratorFunctionPrototype), define(GeneratorFunctionPrototype, "constructor", GeneratorFunction), GeneratorFunction.displayName = define(GeneratorFunctionPrototype, toStringTagSymbol, "GeneratorFunction"), exports.isGeneratorFunction = function (genFun) {
    var ctor = "function" == typeof genFun && genFun.constructor;
    return !!ctor && (ctor === GeneratorFunction || "GeneratorFunction" === (ctor.displayName || ctor.name));
  }, exports.mark = function (genFun) {
    return Object.setPrototypeOf ? Object.setPrototypeOf(genFun, GeneratorFunctionPrototype) : (genFun.__proto__ = GeneratorFunctionPrototype, define(genFun, toStringTagSymbol, "GeneratorFunction")), genFun.prototype = Object.create(Gp), genFun;
  }, exports.awrap = function (arg) {
    return {
      __await: arg
    };
  }, defineIteratorMethods(AsyncIterator.prototype), define(AsyncIterator.prototype, asyncIteratorSymbol, function () {
    return this;
  }), exports.AsyncIterator = AsyncIterator, exports.async = function (innerFn, outerFn, self, tryLocsList, PromiseImpl) {
    void 0 === PromiseImpl && (PromiseImpl = Promise);
    var iter = new AsyncIterator(wrap(innerFn, outerFn, self, tryLocsList), PromiseImpl);
    return exports.isGeneratorFunction(outerFn) ? iter : iter.next().then(function (result) {
      return result.done ? result.value : iter.next();
    });
  }, defineIteratorMethods(Gp), define(Gp, toStringTagSymbol, "Generator"), define(Gp, iteratorSymbol, function () {
    return this;
  }), define(Gp, "toString", function () {
    return "[object Generator]";
  }), exports.keys = function (object) {
    var keys = [];

    for (var key in object) keys.push(key);

    return keys.reverse(), function next() {
      for (; keys.length;) {
        var key = keys.pop();
        if (key in object) return next.value = key, next.done = !1, next;
      }

      return next.done = !0, next;
    };
  }, exports.values = values, Context.prototype = {
    constructor: Context,
    reset: function (skipTempReset) {
      if (this.prev = 0, this.next = 0, this.sent = this._sent = undefined, this.done = !1, this.delegate = null, this.method = "next", this.arg = undefined, this.tryEntries.forEach(resetTryEntry), !skipTempReset) for (var name in this) "t" === name.charAt(0) && hasOwn.call(this, name) && !isNaN(+name.slice(1)) && (this[name] = undefined);
    },
    stop: function () {
      this.done = !0;
      var rootRecord = this.tryEntries[0].completion;
      if ("throw" === rootRecord.type) throw rootRecord.arg;
      return this.rval;
    },
    dispatchException: function (exception) {
      if (this.done) throw exception;
      var context = this;

      function handle(loc, caught) {
        return record.type = "throw", record.arg = exception, context.next = loc, caught && (context.method = "next", context.arg = undefined), !!caught;
      }

      for (var i = this.tryEntries.length - 1; i >= 0; --i) {
        var entry = this.tryEntries[i],
            record = entry.completion;
        if ("root" === entry.tryLoc) return handle("end");

        if (entry.tryLoc <= this.prev) {
          var hasCatch = hasOwn.call(entry, "catchLoc"),
              hasFinally = hasOwn.call(entry, "finallyLoc");

          if (hasCatch && hasFinally) {
            if (this.prev < entry.catchLoc) return handle(entry.catchLoc, !0);
            if (this.prev < entry.finallyLoc) return handle(entry.finallyLoc);
          } else if (hasCatch) {
            if (this.prev < entry.catchLoc) return handle(entry.catchLoc, !0);
          } else {
            if (!hasFinally) throw new Error("try statement without catch or finally");
            if (this.prev < entry.finallyLoc) return handle(entry.finallyLoc);
          }
        }
      }
    },
    abrupt: function (type, arg) {
      for (var i = this.tryEntries.length - 1; i >= 0; --i) {
        var entry = this.tryEntries[i];

        if (entry.tryLoc <= this.prev && hasOwn.call(entry, "finallyLoc") && this.prev < entry.finallyLoc) {
          var finallyEntry = entry;
          break;
        }
      }

      finallyEntry && ("break" === type || "continue" === type) && finallyEntry.tryLoc <= arg && arg <= finallyEntry.finallyLoc && (finallyEntry = null);
      var record = finallyEntry ? finallyEntry.completion : {};
      return record.type = type, record.arg = arg, finallyEntry ? (this.method = "next", this.next = finallyEntry.finallyLoc, ContinueSentinel) : this.complete(record);
    },
    complete: function (record, afterLoc) {
      if ("throw" === record.type) throw record.arg;
      return "break" === record.type || "continue" === record.type ? this.next = record.arg : "return" === record.type ? (this.rval = this.arg = record.arg, this.method = "return", this.next = "end") : "normal" === record.type && afterLoc && (this.next = afterLoc), ContinueSentinel;
    },
    finish: function (finallyLoc) {
      for (var i = this.tryEntries.length - 1; i >= 0; --i) {
        var entry = this.tryEntries[i];
        if (entry.finallyLoc === finallyLoc) return this.complete(entry.completion, entry.afterLoc), resetTryEntry(entry), ContinueSentinel;
      }
    },
    catch: function (tryLoc) {
      for (var i = this.tryEntries.length - 1; i >= 0; --i) {
        var entry = this.tryEntries[i];

        if (entry.tryLoc === tryLoc) {
          var record = entry.completion;

          if ("throw" === record.type) {
            var thrown = record.arg;
            resetTryEntry(entry);
          }

          return thrown;
        }
      }

      throw new Error("illegal catch attempt");
    },
    delegateYield: function (iterable, resultName, nextLoc) {
      return this.delegate = {
        iterator: values(iterable),
        resultName: resultName,
        nextLoc: nextLoc
      }, "next" === this.method && (this.arg = undefined), ContinueSentinel;
    }
  }, exports;
}

function _typeof(obj) {
  "@babel/helpers - typeof";

  return _typeof = "function" == typeof Symbol && "symbol" == typeof Symbol.iterator ? function (obj) {
    return typeof obj;
  } : function (obj) {
    return obj && "function" == typeof Symbol && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj;
  }, _typeof(obj);
}

function asyncGeneratorStep(gen, resolve, reject, _next, _throw, key, arg) {
  try {
    var info = gen[key](arg);
    var value = info.value;
  } catch (error) {
    reject(error);
    return;
  }

  if (info.done) {
    resolve(value);
  } else {
    Promise.resolve(value).then(_next, _throw);
  }
}

function _asyncToGenerator(fn) {
  return function () {
    var self = this,
        args = arguments;
    return new Promise(function (resolve, reject) {
      var gen = fn.apply(self, args);

      function _next(value) {
        asyncGeneratorStep(gen, resolve, reject, _next, _throw, "next", value);
      }

      function _throw(err) {
        asyncGeneratorStep(gen, resolve, reject, _next, _throw, "throw", err);
      }

      _next(undefined);
    });
  };
}

function _classCallCheck(instance, Constructor) {
  if (!(instance instanceof Constructor)) {
    throw new TypeError("Cannot call a class as a function");
  }
}

function _defineProperties(target, props) {
  for (var i = 0; i < props.length; i++) {
    var descriptor = props[i];
    descriptor.enumerable = descriptor.enumerable || false;
    descriptor.configurable = true;
    if ("value" in descriptor) descriptor.writable = true;
    Object.defineProperty(target, descriptor.key, descriptor);
  }
}

function _createClass(Constructor, protoProps, staticProps) {
  if (protoProps) _defineProperties(Constructor.prototype, protoProps);
  if (staticProps) _defineProperties(Constructor, staticProps);
  Object.defineProperty(Constructor, "prototype", {
    writable: false
  });
  return Constructor;
}

function _defineProperty(obj, key, value) {
  if (key in obj) {
    Object.defineProperty(obj, key, {
      value: value,
      enumerable: true,
      configurable: true,
      writable: true
    });
  } else {
    obj[key] = value;
  }

  return obj;
}

function _inherits(subClass, superClass) {
  if (typeof superClass !== "function" && superClass !== null) {
    throw new TypeError("Super expression must either be null or a function");
  }

  subClass.prototype = Object.create(superClass && superClass.prototype, {
    constructor: {
      value: subClass,
      writable: true,
      configurable: true
    }
  });
  Object.defineProperty(subClass, "prototype", {
    writable: false
  });
  if (superClass) _setPrototypeOf(subClass, superClass);
}

function _getPrototypeOf(o) {
  _getPrototypeOf = Object.setPrototypeOf ? Object.getPrototypeOf.bind() : function _getPrototypeOf(o) {
    return o.__proto__ || Object.getPrototypeOf(o);
  };
  return _getPrototypeOf(o);
}

function _setPrototypeOf(o, p) {
  _setPrototypeOf = Object.setPrototypeOf ? Object.setPrototypeOf.bind() : function _setPrototypeOf(o, p) {
    o.__proto__ = p;
    return o;
  };
  return _setPrototypeOf(o, p);
}

function _isNativeReflectConstruct() {
  if (typeof Reflect === "undefined" || !Reflect.construct) return false;
  if (Reflect.construct.sham) return false;
  if (typeof Proxy === "function") return true;

  try {
    Boolean.prototype.valueOf.call(Reflect.construct(Boolean, [], function () {}));
    return true;
  } catch (e) {
    return false;
  }
}

function _assertThisInitialized(self) {
  if (self === void 0) {
    throw new ReferenceError("this hasn't been initialised - super() hasn't been called");
  }

  return self;
}

function _possibleConstructorReturn(self, call) {
  if (call && (typeof call === "object" || typeof call === "function")) {
    return call;
  } else if (call !== void 0) {
    throw new TypeError("Derived constructors may only return object or undefined");
  }

  return _assertThisInitialized(self);
}

function _createSuper(Derived) {
  var hasNativeReflectConstruct = _isNativeReflectConstruct();

  return function _createSuperInternal() {
    var Super = _getPrototypeOf(Derived),
        result;

    if (hasNativeReflectConstruct) {
      var NewTarget = _getPrototypeOf(this).constructor;

      result = Reflect.construct(Super, arguments, NewTarget);
    } else {
      result = Super.apply(this, arguments);
    }

    return _possibleConstructorReturn(this, result);
  };
}

function _superPropBase(object, property) {
  while (!Object.prototype.hasOwnProperty.call(object, property)) {
    object = _getPrototypeOf(object);
    if (object === null) break;
  }

  return object;
}

function _get() {
  if (typeof Reflect !== "undefined" && Reflect.get) {
    _get = Reflect.get.bind();
  } else {
    _get = function _get(target, property, receiver) {
      var base = _superPropBase(target, property);

      if (!base) return;
      var desc = Object.getOwnPropertyDescriptor(base, property);

      if (desc.get) {
        return desc.get.call(arguments.length < 3 ? target : receiver);
      }

      return desc.value;
    };
  }

  return _get.apply(this, arguments);
}

/**
 * 气泡窗类
 * @class
 * 
*/

var Prompt$1 = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 
   * @param {Cesium.Cartesian3 | Array} [opt.position] 弹窗坐标 （type=2时生效）
   * @param {Boolean} opt.show 是否显示 
   * @param {Function} [opt.success] 创建成功的回调函数
   * @param {Number} [opt.type=1] 1~位置变化提示框 / 2~固定坐标提示框
   * @param {Cesium.Cartesian3 | Array} opt.position 固定坐标提示框的坐标（ cartesian3 / [101,30] ），type为1时，可不设置此参数
   * @param {Boolean} [opt.anchor=true] 是否显示锚点
   * @param {Boolean} [opt.closeBtn=true] 是否显示关闭按钮
   * @param {String} opt.className 自定义class
   * @param {String} opt.content 弹窗内容
   * @param {Function} [opt.close] 关闭弹窗时的回调函数
   * @param {Object} [opt.offset] 偏移参数
   * @param {Number} [opt.offset.x] 横坐标偏移像素单位
   * @param {Number} [opt.offset.y] 纵坐标偏移像素单位
   * @param {Object} [opt.style] 弹窗面板样式
   * @param {String} [opt.style.background='white'] 背景色
   * @param {String} [opt.style.boxShadow] 弹窗阴影（css属性）
   * @param {String} [opt.style.color] 弹窗颜色
   * @param {Function} [opt.click] 弹出点击事件
   * 
   */
  function Prompt(viewer, opt) {
    var _this = this;

    _classCallCheck(this, Prompt);

    this.viewer = viewer;
    if (!this.viewer) return;
    this.type = "prompt"; // 默认值

    opt = opt || {};
    var promptType = opt.type == undefined ? 1 : opt.type;
    var defaultOpt = {
      id: new Date().getTime() + "" + Math.floor(Math.random() * 10000),
      type: promptType,
      anchor: promptType == 2 ? true : false,
      closeBtn: promptType == 2 ? true : false,
      offset: promptType == 2 ? {
        x: 0,
        y: -20
      } : {
        x: 10,
        y: 10
      },
      content: "",
      show: true,
      style: {
        background: "rgba(0,0,0,0.5)",
        color: "white"
      },
      client: false // 是否需要自动计算偏移量

    };
    this.opt = Object.assign(defaultOpt, opt);
    /**
     * @property {Object} attr 相关属性
     */

    this.attr = this.opt; // ====================== 创建弹窗内容 start ======================

    var mapid = this.viewer.container.id;
    /**
     * @property {Boolean} isShow 当前显示状态
     */

    this.isShow = this.opt.show == undefined ? true : this.opt.show; // 是否显示

    var anchorHtml = "";
    var closeHtml = "";
    var background = this.opt.style.background;
    var color = this.opt.style.color;

    if (this.opt.anchor) {
      anchorHtml += "\n            <div class=\"prompt-anchor-container\">\n                <div class=\"prompt-anchor\" style=\"background:".concat(background, " !important;\">\n                </div>\n            </div>\n            ");
    }

    if (this.opt.closeBtn) {
      // 移动提示框 不显示关闭按钮
      closeHtml = "<a class=\"prompt-close\" attr=\"".concat(this.opt.id, "\" id=\"prompt-close-").concat(this.opt.id, "\">x</a>");
    }

    var boxShadow = this.opt.style.boxShadow;
    var promptId = "prompt-" + this.opt.id;
    var promptConenet = "\n                <!-- \u6587\u672C\u5185\u5BB9 -->\n                <div class=\"prompt-content-container\" style=\"background:".concat(background, " !important;color:").concat(color, " !important;box-shadow:").concat(boxShadow, " !important\">\n                    <div class=\"prompt-content\" id=\"prompt-content-").concat(this.opt.id, "\">\n                        ").concat(this.opt.content, "\n                    </div>\n                </div>\n                <!-- \u951A -->\n                ").concat(anchorHtml, "\n                <!-- \u5173\u95ED\u6309\u94AE -->\n                ").concat(closeHtml, "\n        "); // 构建弹窗元素 

    this.promptDiv = window.document.createElement("div");
    this.promptDiv.className = "vis3d-prompt ".concat(this.opt.className);
    this.promptDiv.id = promptId;
    this.promptDiv.innerHTML = promptConenet;
    var mapDom = window.document.getElementById(mapid); // 计算地图对象位置 
    // 1、如果是被别人挤下来的 则要加上这个值  
    // 2、如果是自己和父级设置的定位 则不需要加

    this.mapDomRect = this.opt.client ? mapDom.getBoundingClientRect() : {
      top: 0,
      left: 0
    };
    mapDom.appendChild(this.promptDiv);
    var clsBtn = window.document.getElementById("prompt-close-".concat(this.opt.id));
    var that = this; // 绑定弹窗关闭事件

    if (clsBtn) {
      clsBtn.addEventListener("click", function (e) {
        that.hide();
        if (that.opt.close) that.opt.close();
      });
    } // 绑定面板点击事件


    if (this.opt.click) {
      var contentDom = window.document.getElementById("prompt-content-".concat(this.opt.id));
      contentDom.addEventListener('click', function (e) {
        that.opt.click(_this.opt);
      });
    }
    /**
     * @property {Object} promptDom 弹窗div
     */


    this.promptDom = window.document.getElementById(promptId);
    this.position = this.transPosition(this.opt.position); // ====================== 创建弹窗内容 end ======================

    if (promptType == 2) this.bindRender(); // 固定位置弹窗 绑定实时渲染 当到地球背面时 隐藏

    if (this.opt.show == false) this.hide();
    this.containerW = this.viewer.container.offsetWidth;
    this.containerH = this.viewer.container.offsetHeight;
    this.containerLeft = this.viewer.container.offsetLeft;
    this.containerTop = this.viewer.container.offsetTop;
    /**
    * @property {Number} contentW 弹窗宽度
    */

    this.contentW = Math.ceil(Number(this.promptDom.offsetWidth)); // 宽度

    /**
     * @property {Number} contentH 弹窗高度
     */

    this.contentH = this.promptDom.offsetHeight; // 高度

    if (this.opt.success) this.opt.success();
  }
  /**
   * 销毁
   */


  _createClass(Prompt, [{
    key: "destroy",
    value: function destroy() {
      if (this.promptDiv) {
        window.document.getElementById(this.viewer.container.id).removeChild(this.promptDiv);
        this.promptDiv = null;
      }

      if (this.rendHandler) {
        this.rendHandler();
        this.rendHandler = null;
      }
    } // 实时监听

  }, {
    key: "bindRender",
    value: function bindRender() {
      var that = this;
      this.rendHandler = this.viewer.scene.postRender.addEventListener(function () {
        if (!that.isShow && that.promptDom) {
          that.promptDom.style.display = "none";
          return;
        }

        if (!that.position) return;

        if (that.position instanceof Cesium.Cartesian3) {
          var px = Cesium.SceneTransforms.wgs84ToWindowCoordinates(that.viewer.scene, that.position);
          if (!px) return;
          var occluder = new Cesium.EllipsoidalOccluder(that.viewer.scene.globe.ellipsoid, that.viewer.scene.camera.position); // 当前点位是否可见

          var res = occluder.isPointVisible(that.position);

          if (res) {
            if (that.promptDom) that.promptDom.style.display = "block";
          } else {
            if (that.promptDom) that.promptDom.style.display = "none";
          }

          that.setByPX({
            x: px.x,
            y: px.y
          });
        } else {
          that.setByPX({
            x: that.position.x,
            y: that.position.y
          });
        }
      }, this);
    }
    /**
     * 
     * @param {Cesium.Cartesian3 | Object} px 弹窗坐标
     * @param {String} html 弹窗内容
     */

  }, {
    key: "update",
    value: function update(position, html) {
      this.position = position;
      var px = undefined;

      if (position instanceof Cesium.Cartesian3) {
        px = Cesium.SceneTransforms.wgs84ToWindowCoordinates(this.viewer.scene, position.clone());
      } else {
        px = position;
      }

      this.contentW = Math.ceil(Number(this.promptDom.offsetWidth)); // 宽度

      this.contentH = this.promptDom.offsetHeight; // 高度

      if (html) this.setContent(html);
      if (px) this.setByPX(px);
    } // 判断是否在当前视野内

  }, {
    key: "isInView",
    value: function isInView() {
      if (!this.position) return false;
      var px = null;
      var isface = true;

      if (this.position instanceof Cesium.Cartesian2) {
        px = this.position;
      } else {
        px = Cesium.SceneTransforms.wgs84ToWindowCoordinates(this.viewer.scene, this.position); // 是否在地球背面

        var occluder = new Cesium.EllipsoidalOccluder(this.viewer.scene.globe.ellipsoid, this.viewer.scene.camera.position);
        isface = occluder.isPointVisible(this.position);
      }

      if (this.mapDomRect) {
        px.x += this.mapDomRect.left;
        px.y += this.mapDomRect.top;
      }

      var isin = false;

      if (px.x > this.containerLeft && px.x < this.containerLeft + this.containerW && px.y > this.containerTop && px.y < this.containerTop + this.containerH) {
        isin = true;
      }

      return isface && isin;
    }
    /**
     * 是否可见
     * @param {Boolean} isShow true可见，false不可见
     */

  }, {
    key: "setVisible",
    value: function setVisible(isShow) {
      var isin = this.isInView(this.position);

      if (isin && isShow) {
        this.isShow = true;
        if (this.promptDom) this.promptDom.style.display = "block";
      } else {
        this.isShow = false;
        if (this.promptDom) this.promptDom.style.display = "none";
      }
    }
    /**
     * 显示
     */

  }, {
    key: "show",
    value: function show() {
      this.setVisible(true);
    }
    /**
     * 隐藏
     */

  }, {
    key: "hide",
    value: function hide() {
      this.setVisible(false);
    }
    /**
     * 设置弹窗内容
     * @param {String} content 内容 
     */

  }, {
    key: "setContent",
    value: function setContent(content) {
      var pc = window.document.getElementById("prompt-content-".concat(this.opt.id));
      pc.innerHTML = content;
    }
    /**
     * 设置弹窗坐标
     * @param {Object} opt 屏幕坐标
     */

  }, {
    key: "setByPX",
    value: function setByPX(opt) {
      if (!opt) return; // 此处px需要加上实际地图容器的偏移

      if (this.mapDomRect) {
        opt.x += this.mapDomRect.left;
        opt.y += this.mapDomRect.top;
      }

      if (this.promptDom) {
        var contentW = this.promptDom.offsetWidth; // 宽度

        var contentH = this.promptDom.offsetHeight; // 高度

        if (this.opt.type == 1) {
          this.promptDom.style.left = Number(opt.x) + Number(this.opt.offset.x || 0) + "px";
          this.promptDom.style.top = Number(opt.y) + Number(this.opt.offset.y || 0) + "px";
        } else {
          this.promptDom.style.left = Number(opt.x) + Number(this.opt.offset.x || 0) - Number(this.contentW) / 2 + "px";
          this.promptDom.style.top = Number(opt.y) + Number(this.opt.offset.y || 0) - Number(this.contentH) + "px";
        }
      }
    } // 坐标转换

  }, {
    key: "transPosition",
    value: function transPosition(p) {
      var position;

      if (Array.isArray(p)) {
        var posi = Cesium.Cartesian3.fromDegrees(p[0], p[1], p[2] || 0);
        position = posi.clone();
      } else if (p instanceof Cesium.Cartesian3) {
        position = p.clone();
      } else {
        // 像素类型
        position = p;
      }

      return position;
    }
  }]);

  return Prompt;
}();

/**
 * 标绘基类
 * @description 标绘基类，一般不直接实例化，而实例化其子类（见下方Classes）
 * @class
 * @alias BasePlot
 */

var BasePlot = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} style 样式属性 
   */
  function BasePlot(viewer, opt) {
    _classCallCheck(this, BasePlot);

    this.viewer = viewer;
    this.opt = opt || {};
    /**
     * @property {Object} style 样式
     */

    this.style = this.opt.style || {};
    /**
     * @property {String | Number} objId 唯一id
     */

    this.objId = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    this.handler = undefined;
    this.modifyHandler = undefined;
    /**
     * @property {String} type 类型
     */

    this.type = '';
    /**
     *@property {Cesium.Cartesian3[]} positions 坐标数组
     */

    this.positions = [];
    /**
     *@property {String} state 标识当前状态 no startCreate creating endCreate startEdit endEdit editing
     */

    this.state = null; //

    /**
     * @property {Object} prompt 鼠标提示框
     */

    this.prompt = null; // 初始化鼠标提示框

    this.controlPoints = []; // 控制点

    this.modifyPoint = null;
    /**
     * 图标entity对象
     * @property {Cesium.Entity} entity entity对象
    */

    this.entity = null;
    this.pointStyle = {};
    /**
     * @property {Object} promptStyle 鼠标提示框样式
     */

    this.promptStyle = this.opt.prompt || {
      show: true
    };
    this.properties = {}; // 缩放分辨率比例

    this.scale = this.opt.scale || [1, 1];

    this.initDragEvent();
  }
  /**
   * 
   * @param {Object} px 像素坐标 
   * @returns {Cesium.Cartesian3} 世界坐标
   */


  _createClass(BasePlot, [{
    key:"initDragEvent",
    value:function initDragEvent(){
      let that = this;
      if (!this.modifyPositionHandler) this.modifyPositionHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.modifyPositionHandler.setInputAction(function (evt) {
        //当没有实体或预览状态
        if (!that.entity) return;
        let pick = that.viewer.scene.pick(evt.position);  
        // console.log("所有图元类型",that, pick);
        if (Cesium.defined(pick) && pick.id&&that.objId==pick.id.objId&&(that.state == "startEdit" || that.state == "editing")) {
          console.log("点击的图元类型",that, pick,that.type);
          //初始点
           let initPosition = that.getCatesian3FromPX(evt.position, that.viewer);
          // let initPosition = that.viewer.camera.pickEllipsoid(evt.position, that.viewer.scene.globe.ellipsoid);
          that.forbidDrawWorld(true);
          that.modifyPositionHandler.setInputAction(function (ev) {
            let endPosition = that.getCatesian3FromPX(ev.endPosition, that.viewer);
            // let endPosition = that.viewer.camera.pickEllipsoid(ev.endPosition, that.viewer.scene.globe.ellipsoid);
            //移动的距离
            let moveDistace =new Cesium.Cartesian3()
            Cesium.Cartesian3.subtract(endPosition.clone(), initPosition.clone(), moveDistace); 

           let [wgs1,wgs2]=util$1.cs2jwds([endPosition.clone(),initPosition.clone()],that.viewer);
           let moveDistanc_wgs=wgs1.map((m,n)=>{
            if(n>1)return m
            else return m-wgs2[n] 
           })
           switch (that.type) {
            case 'circle':
              if (that.floatPoint) that.floatPoint.show = true;
              if (that.centerPoint) that.centerPoint.show = true;
              let centerPosition = Cesium.Cartesian3.add(that.position, moveDistace, new Cesium.Cartesian3());  
              let  centerPoint=util$1.updatePositionsHeight([centerPosition],0)[0];  
              //  centerPoint = Cesium.Cartesian3.add(that.position, moveDistace, new Cesium.Cartesian3());  
              let floatPosition = Cesium.Cartesian3.add(that.floatPosition, moveDistace, new Cesium.Cartesian3());
              let floatPoint=util$1.updatePositionsHeight([floatPosition],0)[0];
              //  floatPoint = Cesium.Cartesian3.add(that.floatPosition, moveDistace, new Cesium.Cartesian3());
              that.position = centerPoint;  
              that.entity.position.setValue(that.position);//圆实体
              that.centerPoint.position.setValue(that.position);//点实体
              that.floatPosition = floatPoint; 
              that.floatPoint.position.setValue(that.floatPosition);//点实体
              initPosition=endPosition;    
              break; 
              case 'label':
              case 'billboard': 
              let  billboardPosition= Cesium.Cartesian3.add(that.position, moveDistace, new Cesium.Cartesian3());
              let  billboardPoint=util$1.updatePositionsHeight([billboardPosition],0)[0];
              that.entity.position.setValue(billboardPoint.clone());
              that.position = billboardPoint.clone();
              initPosition=endPosition;    
              break; 
              case 'polygon':
              case 'polyline':
              case 'arrow':
                /**
                 * @property {String} arrowType 箭头类型（1~攻击箭头/2~攻击箭头平尾/3~攻击箭头燕尾/4~闭合曲面/5~钳击箭头/6~单尖直箭头/7~粗单尖直箭头/8~集结地/9~弓形面/10~粗直箭头/11~矩形棋/12~扇形/13~三角旗/14~曲线旗/15~曲线/16~单线箭头）
                 */
                    // console.log("攻击箭头燕尾",that,that.arrowType);
                    //移动点
                    for (var i = 0; i < that.controlPoints.length; i++) {
                      var point = that.controlPoints[i]; 
                      let pointPosition= that.positions[point.wz];   
                     /*Cesium.Cartesian3.add(pointPosition, moveDistace,pointPosition);   
                       let pointPosition_h=util$1.updatePositionsHeight([newPointPosition],0)[0]; 
                       point.position.setValue(pointPosition_h); */
                        
                        let trasformPosition=util$1.c2jwd(pointPosition);
                        let newpoint=trasformPosition.map((m,n)=>{
                          if(n>1)return m
                          else return m+moveDistanc_wgs[n] 
                        })
                        Cesium.Cartesian3.clone(Cesium.Cartesian3.fromDegrees(...newpoint),pointPosition);  
                        point.position.setValue(Cesium.Cartesian3.fromDegrees(...newpoint));
                        // console.log("所有又箭头的点位信息",point,point.position);
                      } 
                    initPosition=endPosition;   
              break; 
              case 'rectangle':
               /*  let leftPoint = Cesium.Cartesian3.add(that.leftup, moveDistace, new Cesium.Cartesian3());  
                let leftUpPoint=util$1.updatePositionsHeight([leftPoint],0)[0];  
                //  leftUpPoint = Cesium.Cartesian3.add(that.leftup, moveDistace, new Cesium.Cartesian3());  
                let rightPoint = Cesium.Cartesian3.add(that.rightdown, moveDistace, new Cesium.Cartesian3());
                let rightdownPoint=util$1.updatePositionsHeight([rightPoint],0)[0];
                //  rightdownPoint = Cesium.Cartesian3.add(that.rightdown, moveDistace, new Cesium.Cartesian3()) 
                that.leftup = leftUpPoint; 
                that.leftupPoint.position.setValue(leftUpPoint);
                that.rightdown = rightdownPoint; 
                that.rightdownPoint.position.setValue(rightdownPoint); */
                [that.leftup,that.rightdown].forEach((item,index)=>{
                  let trasformPosition=util$1.c2jwd(item);
                  let [lng, lat, hei]=trasformPosition.map((m,n)=>{
                    if(n>1)return m
                    else return m+moveDistanc_wgs[n] 
                    })        
                  if(index==0){
                    let leftUpPoint=Cesium.Cartesian3.fromDegrees(lng, lat, hei);
                    that.leftup = leftUpPoint; 
                    that.leftupPoint.position.setValue(leftUpPoint);
                  }else{
                    let rightdownPoint=Cesium.Cartesian3.fromDegrees(lng, lat, hei);
                    that.rightdown = rightdownPoint; 
                    that.rightdownPoint.position.setValue(rightdownPoint);
                  }
                })
                initPosition=endPosition;   
                break;
            default:
              break;
           }
          }, Cesium.ScreenSpaceEventType.MOUSE_MOVE); 
        } 
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyPositionHandler.setInputAction(function (evt) { 
        that.forbidDrawWorld(false);  
        that.modifyPositionHandler.removeInputAction(Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, 
{
    key: "getCatesian3FromPX",
    value: function getCatesian3FromPX(px) {
      if (!px) return;
      px.x = px.x / this.scale[0];
      px.y = px.y / this.scale[1];
      var picks = this.viewer.scene.drillPick(px);
      this.viewer.scene.render();
      var cartesian;
      var isOn3dtiles = false;

      for (var i = 0; i < picks.length; i++) {
        if (picks[i] && picks[i].primitive && picks[i].primitive instanceof Cesium.Cesium3DTileset) {
          //模型上拾取
          isOn3dtiles = true;
          break;
        }
      }

      if (isOn3dtiles) {
        cartesian = this.viewer.scene.pickPosition(px);
      } else {
        var ray = this.viewer.camera.getPickRay(px);
        if (!ray) return null;
        cartesian = this.viewer.scene.globe.pick(ray, this.viewer.scene);
        //sin add 20240718
        if (!cartesian) {
          cartesian = this.viewer.camera.pickEllipsoid(
              px,
              this.viewer.scene.globe.ellipsoid
          );
        }
      }

      return cartesian;
    }
    /**
     * 
     * @returns {Cesium.Entity} 实体对象
     */

  }, {
    key: "getEntity",
    value: function getEntity() {
      return this.entity;
    }
    /**
     * 
     * @param {Boolean} isWgs84 是否转化为经纬度
     * @returns {Array} 坐标数组
     */

  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      return isWgs84 ? util$1.cartesiansToLnglats(this.positions, this.viewer) : this.positions;
    }
    /**
    * 获取经纬度坐标
    * @returns {Array} 经纬度坐标数组
    */

  }, {
    key: "getLnglats",
    value: function getLnglats() {
      return util$1.cartesiansToLnglats(this.positions, this.viewer);
    }
    /**
     * 设置自定义属性
     * @param {Object} prop 属性 
     */

  }, {
    key: "setOwnProp",
    value: function setOwnProp(prop) {
      if (this.entity) this.entity.ownProp = prop;
    }
    /**
     * 移除当前entity对象
     */

  }, {
    key: "remove",
    value: function remove() {
      if (this.entity) {
        this.state = "no";
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }
    }
    /**
     * 设置entity对象的显示隐藏
     * @param {Boolean} visible 
     */

  }, {
    key: "setVisible",
    value: function setVisible(visible) {
      if (this.entity) this.entity.show = visible;
    } // 操作控制

  }, {
    key: "forbidDrawWorld",
    value: function forbidDrawWorld(isForbid) {
      this.viewer.scene.screenSpaceCameraController.enableRotate = !isForbid;
      this.viewer.scene.screenSpaceCameraController.enableTilt = !isForbid;
      this.viewer.scene.screenSpaceCameraController.enableTranslate = !isForbid;
      this.viewer.scene.screenSpaceCameraController.enableInputs = !isForbid;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }
      
      if (this.modifyPositionHandler) {
        this.modifyPositionHandler.destroy();
        this.modifyPositionHandler = null;
      }

      if (this.entity) {
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }

      this.positions = [];
      this.style = null;

      for (var i = 0; i < this.controlPoints.length; i++) {
        var point = this.controlPoints[i];
        this.viewer.entities.remove(point);
      }

      this.controlPoints = [];
      this.modifyPoint = null;

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      this.state = "no";
      this.forbidDrawWorld(false);
    }
    /**
     * 
     * 开始编辑
     */

  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (this.state == "startEdit" || this.state == "editing" || !this.entity) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.entity.show = true;
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.entity) return;
        var pick = that.viewer.scene.pick(evt.position);
        if (Cesium.defined(pick) && pick.id && pick.collection) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, [that.entity, that.modifyPoint]);

        if (cartesian) {
          that.modifyPoint.position.setValue(cartesian);
          that.positions[that.modifyPoint.wz] = cartesian;
          that.state = "editing";
          if (callback) callback();
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.forbidDrawWorld(false);
        that.state = "editing";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
    /**
     * 结束编辑
     * @param {Function} callback 回调函数
     * @example
     *  plotObj.endEdit(function(entity){})
     */

  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      for (var i = 0; i < this.controlPoints.length; i++) {
        var point = this.controlPoints[i];
        if (point) point.show = false;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.forbidDrawWorld(false);
      this.state = "endEdit";
    }
    /**
     * 结束创建
     */

  }, {
    key: "endCreate",
    value: function endCreate() {}
    /**
     * 在当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {} // 构建控制点

  }, {
    key: "createPoint",
    value: function createPoint(position) {
      if (!position) return;
      this.pointStyle.color = this.pointStyle.color || Cesium.Color.CORNFLOWERBLUE;
      this.pointStyle.outlineColor = this.pointStyle.color || Cesium.Color.CORNFLOWERBLUE;
      var color = this.pointStyle.color instanceof Cesium.Color ? this.pointStyle.color : Cesium.Color.fromCssColorString(this.pointStyle.color);
      color = color.withAlpha(this.pointStyle.colorAlpha || 1);
      var outlineColor = this.pointStyle.outlineColor instanceof Cesium.Color ? this.pointStyle.outlineColor : Cesium.Color.fromCssColorString(this.pointStyle.outlineColor);
      outlineColor = outlineColor.withAlpha(this.pointStyle.outlineColorAlpha || 1);
      return this.viewer.entities.add({
        position: position,
        point: {
          pixelSize: this.pointStyle.property || 10,
          color: color,
          outlineWidth: this.pointStyle.outlineWidth || 0,
          outlineColor: outlineColor,
          disableDepthTestDistance: Number.POSITIVE_INFINITY
        },
        show: false
      });
    } // 获取当前标绘的样式

    /*  getStyle() {
        if (!this.entity) return;
        let graphic = this.entity[this.plotType];
        if (!graphic) return;
        let style = {};
        switch (this.plotType) {
            case 'polyline':
                style.clampToGround = graphic.clampToGround._value; // 是否贴地
                style.distanceDisplayCondition = graphic.distanceDisplayCondition._value; // 显示控制
                style.width = graphic.width._value; // 线宽
                let colorObj = this.transfromLineMaterial(graphic.material);
                style = Object.assign(style, colorObj);
                break;
            case "polygon":
                style.heightReference = graphic.heightReference.getValue();
                style.fill = graphic.fill._value;
                style.extrudedHeight = graphic.extrudedHeight._value;
                let gonColorObj = this.transfromGonMaterial(graphic.material);
                style = Object.assign(style, gonColorObj);
                  style.outline = graphic.outline._value;
                let ocv = graphic.outlineColor.getValue();
                style.outlineColorAlpha = ocv.alpha;
                style.outlineColor = new Cesium.Color(ocv.red, ocv.green, ocv.blue, 1).toCssHexString();
                  break;
            default:
                break;
        }
        return style;
    } */
    // 获取线的材质

  }, {
    key: "transfromLineMaterial",
    value: function transfromLineMaterial(material) {
      if (!material) return;
      var colorObj = {};

      if (material instanceof Cesium.Color) {
        var colorVal = material.color.getValue();
        colorObj.colorAlpha = colorVal.alpha; // 转为hex

        colorObj.colorHex = new Cesium.Color(colorVal.red, colorVal.green, colorVal.blue, 1).toCssHexString();
      }

      return colorObj;
    } // 获取面材质

  }, {
    key: "transfromGonMaterial",
    value: function transfromGonMaterial(material) {
      if (!material) return;
      var colorObj = {};

      if (material instanceof Cesium.Color) {
        var colorVal = material.color.getValue();
        colorObj.colorAlpha = colorVal.alpha; // 转为hex

        colorObj.colorHex = new Cesium.Color(colorVal.red, colorVal.green, colorVal.blue, 1).toCssHexString();
      }

      return colorObj;
    } // 设置实体的属性

  }, {
    key: "setAttr",
    value: function setAttr(attr) {
      this.properties.attr = attr || {};
    }
  }, {
    key: "getAttr",
    value: function getAttr() {
      return this.properties.attr;
    }
    /**
     * 缩放至当前绘制的对象
    */

  }, {
    key: "zoomTo",
    value: function zoomTo() {
      if (this.entity) {
        this.viewer.zoomTo(this.entity);
      }
    }
  }]);

  return BasePlot;
}();

/**
 * 图标标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreateBillboard
 */

var CreateBillboard = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreateBillboard, _BasePlot);

  var _super = _createSuper(CreateBillboard);

  function CreateBillboard(viewer, opt) {
    var _this;

    _classCallCheck(this, CreateBillboard);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.style = _this.opt.style;
    _this.type = "billboard";
    _this.viewer = viewer;
    var defaultStyle = {
      verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
      scale: 1
    };
    _this.style = Object.assign({}, defaultStyle, _this.style || {});
    _this.entity = null;

    if (!_this.style.hasOwnProperty("image")) {
      console.log("未设置billboard的参数！");
    }
    /**
     * @property {Cesium.Cartesian3} 图标坐标
     */


    _this.position = null;
    return _this;
  }
  /**
   * 开始绘制
   * @param {Function} callback 绘制成功后回调函数
  */


  _createClass(CreateBillboard, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;
        that.position = cartesian.clone();
        that.entity = that.createBillboard(that.position);

        if (that.handler) {
          that.handler.destroy();
          that.handler = null;
        }

        if (that.prompt) {
          that.prompt.destroy();
          that.prompt = null;
        }

        that.state = "endCreate";
        if (callback) callback(that.entity);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.prompt.update(evt.endPosition, "单击新增");
        that.state = "creating";
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
    /**
     * 结束绘制
     * @param {Function} callback 结束绘制后回调函数
    */

  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
    /**
     * 通过坐标数组构建
     * @param {Array} lnglatArr 经纬度坐标数组
     * @callback {Function} callback 绘制成功后回调函数
    */

  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      if (!lnglatArr) return;
      this.state = "startCreate";
      var position = null;

      if (lnglatArr instanceof Cesium.Cartesian3) {
        position = lnglatArr.clone();
      } else {
        position = Cesium.Cartesian3.fromDegrees(Number(lnglatArr[0]), Number(lnglatArr[1]), Number(lnglatArr[2] || 0));
      }

      if (!position) return;
      this.position = position.clone();
      this.entity = this.createBillboard(this.position);
      if (callback) callback(this.entity);
      this.state = "endCreate";
    }
    /**
     * 设置相关样式
     * @param {Object} style 样式 
     */

  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      var billboard = this.entity.billboard;
      if (style.image != undefined) billboard.image = style.image;

      if (style.heightReference != undefined) {
        var heightReference = 1;

        if (this.style.heightReference == true) {
          heightReference = 1;
        } else {
          heightReference = this.style.heightReference;
        }

        billboard.heightReference = heightReference;
      }

      if (style.heightReference != undefined) billboard.heightReference = style.heightReference == undefined ? 1 : Number(this.style.heightReference); // 如果直接设置为true 会导致崩溃

      if (style.scale != undefined) billboard.scale = Number(style.scale);

      if (style.color) {
        var color = style.color instanceof Cesium.Color ? style.color : Cesium.Color.fromCssColorString(style.color);
        color = color.withAlpha(style.colorAlpha || 1);
        billboard.color = color;
      }

      this.style = Object.assign(this.style, style);
    }
    /**
     * 获取样式
     * @returns {Object} 样式
    */

  }, {
    key: "getStyle",
    value: function getStyle() {
      var obj = {};
      var billboard = this.entity.billboard;
      obj.image = this.style.image;

      if (billboard.heightReference) {
        var heightReference = billboard.heightReference.getValue();
        obj.heightReference = Number(heightReference);
      }

      obj.scale = billboard.scale.getValue();

      if (billboard.color) {
        var color = billboard.color.getValue();
        obj.colorAlpha = color.alpha;
        obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();
      }

      return obj;
    }
    /**
     * 开始编辑 
     */

  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (this.state == "startEdit" || this.state == "editing" || !this.entity) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;
      var editBillboard;
      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);

        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          editBillboard = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        //移动时绘制线
        if (!editBillboard) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        editBillboard.position.setValue(cartesian.clone());
        that.position = cartesian.clone();
        that.state = "editing";
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        //移动时绘制线
        if (!editBillboard) return;
        that.forbidDrawWorld(false);

        if (that.modifyHandler) {
          that.modifyHandler.destroy();
          that.modifyHandler = null;
          that.state = "editing";
        }
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
    /**
       * 结束编辑
       * @param {Function} callback 回调函数
       */

  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.state = "endEdit";
    }
  }, {
    key: "createBillboard",
    value: function createBillboard(cartesian) {
      if (!cartesian) return;
      var billboard = this.viewer.entities.add({
        position: cartesian,
        billboard: {
          color: this.style.color ? this.style.color instanceof Cesium.Color ? this.style.color : Cesium.Color.fromCssColorString(this.style.color).withAlpha(this.style.colorAlpha || 1) : Cesium.Color.WHITE,
          image: this.style.image || "../img/mark4.png",
          scale: this.style.scale || 1,
          pixelOffset: this.style.pixelOffset,
          heightReference: this.style.heightReference == undefined ? Cesium.HeightReference.NONE : Number(this.style.heightReference),
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          translucencyByDistance: new Cesium.NearFarScalar(10.0e6, 1.0, 20.0e6, 0.0),//sin_add 20241018
        }
      });
      billboard.objId = this.objId;
      return billboard;
    }
    /**
     * 移除
    */

  }, {
    key: "remove",
    value: function remove() {
      if (this.entity) {
        this.state = "no";
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }
    }
  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      var lnglats = isWgs84 ? util$1.cartesianToLnglat(this.position) : this.position;
      return [lnglats]; // 包装一层 和其他标会类保持一致
    }
    /**
     * 设置图标坐标
     * @param {Cesium.Cartesian3 | Array} p 坐标
    */

  }, {
    key: "setPosition",
    value: function setPosition(p) {
      var position = null;

      if (p instanceof Cesium.Cartesian3) {
        position = p;
      } else {
        position = Cesium.Cartesian3.fromDegrees(p[0], p[1], p[2] || 0);
      }

      this.entity.position.setValue(position.clone());
      this.position = position.clone();
    }
  }]);

  return CreateBillboard;
}(BasePlot);

/**
 * 圆标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreateCircle
 */

var CreateCircle = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreateCircle, _BasePlot);

  var _super = _createSuper(CreateCircle);

  function CreateCircle(viewer, opt) {
    var _this;

    _classCallCheck(this, CreateCircle);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.type = "circle";
    _this.objId = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    _this.viewer = viewer;
    _this.floatPoint = null;
    /**
     * @property {Cesium.Entity} centerPoint 圆中心点
     */

    _this.centerPoint = null;
    /**
     * @property {Cesium.Cartesian3} position 圆中心点坐标
     */

    _this.position = null;
    _this.floatPosition = null;
    /**
     * @property {Number} 圆半径
     */

    _this.radius = 0.001;
    _this.modifyPoint = null;
    _this.pointArr = [];
    _this.entityPolyline = null;
    return _this;
  }
  /**
   * 开始绘制
   * @param {Function} callback 绘制成功后回调函数
  */


  _createClass(CreateCircle, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (!that.centerPoint) {
          that.position = cartesian;
          that.centerPoint = that.createPoint(cartesian);
          that.centerPoint.typeAttr = "center";
          that.floatPoint = that.createPoint(cartesian.clone());
          that.floatPosition = cartesian.clone();
          that.floatPoint.typeAttr = "float";
          that.entity = that.createCircle(1);
          that.entityPolyline =that.createEntityPolyline(1)
        } else {
          if (that.entity) {
            that.endCreate();
            if (callback) callback(that.entity);
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        // 移动时绘制线
        if (!that.centerPoint) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          return;
        }

        that.state = "creating";
        that.prompt.update(evt.endPosition, "再次单击结束");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (that.floatPoint) {
          that.floatPoint.position.setValue(cartesian);
          that.floatPosition = cartesian.clone();
        }

        that.radius = Cesium.Cartesian3.distance(cartesian, that.position);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
    /**
     * 通过坐标数组构建
     * @param {Array} lnglatArr 经纬度坐标数组
     * @callback {Function} callback 绘制成功后回调函数
    */

  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      if (!lnglatArr || lnglatArr.length < 1) return;
      this.state = "startCreate";

      if (Array.isArray(lnglatArr)) {
        // 第一种 传入中间点坐标和边界上某点坐标
        var isCartesian3 = lnglatArr[0] instanceof Cesium.Cartesian3;
        var positions = [];

        if (isCartesian3) {
          positions = lnglatArr;
        } else {
          positions = util$1.lnglatsToCartesians(lnglatArr);
        }

        if (!positions || positions.length < 1) return;
        this.position = positions[0].clone();
        this.radius = Cesium.Cartesian3.distance(this.position, positions[1]);
        this.floatPosition = positions[1].clone();
      } else {
        // 第二种 传入中间点坐标和半径
        this.position = lnglatArr.position;
        this.radius = lnglatArr.radius;
        this.floatPosition = util$1.getPositionByLength();
      }

      this.centerPoint = this.createPoint(this.position);
      this.centerPoint.typeAttr = "center";
      this.floatPoint = this.createPoint(this.floatPosition);
      this.floatPoint.typeAttr = "float";
      this.entity = this.createCircle();
      this.entityPolyline =this.createEntityPolyline()
      this.state = "endCreate";
      if (callback) callback(this.entity);
    }
    /**
      * 开始编辑
      * @param {Function} callback 回调函数
      */

  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (this.state == "startEdit" || this.state == "editing" || !this.entity) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;
      if (that.floatPoint) that.floatPoint.show = true;
      if (that.centerPoint) that.centerPoint.show = true;
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.entity) return;
        that.state = "editing";
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        } else {
          if (that.floatPoint) that.floatPoint.show = false;
          if (that.centerPoint) that.centerPoint.show = false;

          if (that.modifyHandler) {
            that.modifyHandler.destroy();
            that.modifyHandler = null;
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);

      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.state = "editing";

        if (that.modifyPoint.typeAttr == "center") {
          // 计算当前偏移量
          var subtract = Cesium.Cartesian3.subtract(cartesian, that.position, new Cesium.Cartesian3());
          that.position = cartesian;
          that.centerPoint.position.setValue(that.position);
          that.entity.position.setValue(that.position);
          that.floatPosition = Cesium.Cartesian3.add(that.floatPosition, subtract, new Cesium.Cartesian3());
          that.floatPoint.position.setValue(that.floatPosition);
        } else {
          that.floatPosition = cartesian;
          that.floatPoint.position.setValue(that.floatPosition);
          that.radius = Cesium.Cartesian3.distance(that.floatPosition, that.position);
        }

        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.forbidDrawWorld(false);
        that.state = "editing";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
    /**
     * 结束绘制cartesiansToLnglats
     * @param {Function} callback 结束绘制后回调函数
    */

  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.state = "endCreate";

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.floatPoint) that.floatPoint.show = false;
      if (that.centerPoint) that.centerPoint.show = false;

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
    /**
      * 结束编辑
      * @param {Function} callback 回调函数
      */

  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      if (this.floatPoint) this.floatPoint.show = false;
      if (this.centerPoint) this.centerPoint.show = false;

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.forbidDrawWorld(false);
      this.state = "endEdit";
    }
  }, 
  {
    key: "changeProperty",
    value: function changeProperty(code) {
      var that = this
      if(!that.entity) return
      if(code == 1) {
        let semiMajorAxis = new Cesium.CallbackProperty(function () {
          return that.radius;
        }, false)
        let semiMinorAxis =  new Cesium.CallbackProperty(function () {
          return that.radius;
        }, false)
        that.entity.ellipse.semiMajorAxis = semiMajorAxis
        that.entity.ellipse.semiMinorAxis = semiMinorAxis
      }else {
        that.entity.ellipse.semiMajorAxis = that.radius
        that.entity.ellipse.semiMinorAxis = that.radius
      }

      if(!that.entityPolyline) return
      if(code == 1) {
        let positions = new Cesium.CallbackProperty(function () {
          return util$1.getCirclePointsByRadius({center:that.position,radius:that.radius+10,angle:5})
        }, false)
        that.entityPolyline.polyline.positions = positions
      }else {
        let positions =util$1.getCirclePointsByRadius({center:that.position,radius:that.radius+10,angle:5})
        that.entityPolyline.polyline.positions = positions
      }

    }
  },
  {
    key: "createCircle",
    value: function createCircle(code) {
      var that = this;
      let semiMajorAxis,semiMinorAxis
      if(code) {
        semiMajorAxis = new Cesium.CallbackProperty(function () {
          return that.radius;
        }, false)
        semiMinorAxis =  new Cesium.CallbackProperty(function () {
          return that.radius;
        }, false)
      }else {
        semiMajorAxis = that.radius;
        semiMinorAxis = that.radius;
      }
      var defauteObj = {
        semiMajorAxis: semiMinorAxis,
        semiMinorAxis: semiMinorAxis,
        material: this.style.color instanceof Cesium.Color ? this.style.color : this.style.color ? Cesium.Color.fromCssColorString(this.style.color).withAlpha(this.style.colorAlpha || 1) : Cesium.Color.WHITE,
        outlineColor: this.style.outlineColor instanceof Cesium.Color ? this.style.outlineColor : this.style.outlineColor ? Cesium.Color.fromCssColorString(this.style.outlineColor).withAlpha(this.style.outlineColorAlpha || 1) : Cesium.Color.BLACK,
        outline: this.style.outline,
        heightReference: this.style.heightReference,
        outlineWidth: this.style.outlineWidth,
        fill: this.style.fill,
        height:0
      };
      /*  if (
         !this.style.heightReference ||
         Number(this.style.heightReference) == 0
       ) {
         defauteObj.height = 100 || this.style.height;
         defauteObj.heightReference = 0;
       } else {
         defauteObj.heightReference = 1;
       } */

      var ellipse = this.viewer.entities.add({
        position: this.position,
        ellipse: defauteObj
      });
      ellipse.objId = this.objId;
      return ellipse;
    }
  },
  {
    key: "createEntityPolyline",
    value: function createEntityPolyline(code) {
      var that = this;
       let positions
       if(code) {
        positions = new Cesium.CallbackProperty(function () {
          return util$1.getCirclePointsByRadius({center:that.position,radius:that.radius+10,angle:5})
        }, false)
      }else {
        positions=util$1.getCirclePointsByRadius({center:that.position,radius:that.radius+10,angle:5})
      }
       const poltlineObj = {
        positions: positions,
        material: this.style.outlineColor instanceof Cesium.Color ? this.style.outlineColor : this.style.outlineColor ? Cesium.Color.fromCssColorString(this.style.outlineColor).withAlpha(this.style.outlineColorAlpha || 1) : Cesium.Color.WHITE,
        show: this.style.outline,
        width: this.style.outlineWidth,
        clampToGround:true,
        zIndex :1000
      };

      var EntityPolyline = this.viewer.entities.add({
        position: this.position,
        polyline:poltlineObj
      });
      EntityPolyline.objId = this.objId;
      return EntityPolyline;
    }
  },
   {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      var color = Cesium.Color.fromCssColorString(style.color || "#ffff00");
      color = color.withAlpha(style.colorAlpha);
      this.entity.ellipse.material = color;
      this.entity.ellipse.outline = style.outline || false;
      this.entity.ellipse.outlineWidth = style.outlineWidth;
      var outlineColor = Cesium.Color.fromCssColorString(style.outlineColor || "#000000");
      outlineColor = outlineColor.withAlpha(style.outlineColorAlpha);
      this.entity.ellipse.outlineColor = outlineColor;
      this.entity.ellipse.heightReference = Number(style.heightReference);

      this.entityPolyline.polyline.width = style.outlineWidth
      this.entityPolyline.polyline.material = outlineColor
      this.entityPolyline.polyline.show = style.outline

      // if (style.heightReference == 0) {
      //   this.entity.ellipse.height = Number(style.height);
      //   this.updatePointHeight(style.height);
      // }

      this.entity.ellipse.fill = Boolean(style.fill);
      this.entity.ellipse.height = 0
      this.style = Object.assign(this.style, style);
    }
  }, {
    key: "getStyle",
    value: function getStyle() {
      var obj = {};
      var ellipse = this.entity.ellipse;
      // console.log(1, ellipse.material.color.getValue())
      // if (ellipse.outline) console.log(2, ellipse.outline.getValue())
      // console.log(3, ellipse.outlineColor.getValue())
      // if (ellipse.height) console.log(4, ellipse.height.getValue())
      // if (ellipse.fill) console.log(5, ellipse.fill.getValue())
      // if (ellipse.fill) console.log(6, ellipse.heightReference.getValue())
      var color = ellipse.material?.color?.getValue()||ellipse.material._color._value;
      obj.colorAlpha = color.alpha;
      obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();
      obj.outline = ellipse.outline.getValue();
      if (ellipse.outline._value) {
        obj.outlineWidth = ellipse.outlineWidth.getValue();
        var outlineColor = ellipse.outlineColor.getValue();
        obj.outlineColorAlpha = outlineColor.alpha || 1;
        obj.outlineColor = new Cesium.Color(outlineColor.red, outlineColor.green, outlineColor.blue, 1).toCssHexString();
      }
      if (ellipse.height) obj.height = ellipse.height.getValue();
      if (ellipse.fill){
        obj.fill = ellipse.fill.getValue();
      }else{
        obj.fill = false
      }
      obj.height = 0
      // obj.heightReference = ellipse.heightReference.getValue();
      return obj;
    }
  }, {
    key: "destroy",
    value: function destroy() {
      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      if (this.entity) {
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }

      if(this.entityPolyline){
        this.viewer.entities.remove(this.entityPolyline);
        this.entityPolyline = null;
      }

      if (this.floatPoint) {
        this.viewer.entities.remove(this.floatPoint);
        this.floatPoint = null;
      }

      if (this.centerPoint) {
        this.viewer.entities.remove(this.centerPoint);
        this.centerPoint = null;
      }

      this.style = null;
      this.modifyPoint = null;
      if (this.prompt) this.prompt.destroy();
      this.forbidDrawWorld(false);
      this.state = "no";
    } // 修改点的高度

  }, {
    key: "updatePointHeight",
    value: function updatePointHeight(h) {
      var centerP = this.centerPoint.position.getValue();
      var floatP = this.floatPoint.position.getValue();
      centerP = util$1.updatePositionsHeight([centerP], Number(this.style.height))[0];
      floatP = util$1.updatePositionsHeight([floatP], Number(this.style.height))[0];
      this.centerPoint.position.setValue(centerP);
      this.floatPoint.position.setValue(floatP);
    }
  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      var positions = [];

      if (isWgs84) {
        positions = util$1.cartesiansToLnglats([this.position, this.floatPosition], this.viewer);
      } else {
        positions = [this.position, this.floatPosition];
      }

      return positions;
    }
  }]);

  return CreateCircle;
}(BasePlot);

/**
 * 小模型（gltf、glb）标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreateGltfModel
 */

var CreateGltfModel = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreateGltfModel, _BasePlot);

  var _super = _createSuper(CreateGltfModel);

  function CreateGltfModel(viewer, opt) {
    var _this;

    _classCallCheck(this, CreateGltfModel);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.type = "gltfModel";
    _this.viewer = viewer;
    var defaultStyle = {
      heading: 0,
      pitch: 0,
      roll: 0,
      minimumPixelSize: 24,
      maximumScale: 120
    };
    _this.style = Object.assign(defaultStyle, _this.opt.style || {});

    if (!_this.style.uri) {
      console.warn("请输入模型地址！");
      return _possibleConstructorReturn(_this);
    }
    /**
     * @property {String} modelUri 模型地址
     */


    _this.modelUri = _this.style.uri;
    _this.entity = null;
    return _this;
  }

  _createClass(CreateGltfModel, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);

        if (cartesian) {
          that.entity.position = cartesian;
          that.position = cartesian.clone();
        }

        that.endCreate();
        if (callback) callback(that.entity);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.prompt.update(evt.endPosition, "单击新增");
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, [that.entity]);
        if (!cartesian) return;

        if (!that.entity) {
          that.entity = that.createGltfModel(cartesian.clone());
        } else {
          that.entity.position = cartesian;
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      if (!lnglatArr) return;
      this.state = "startCreate";

      if (lnglatArr instanceof Cesium.Cartesian3) {
        this.position = lnglatArr;
      } else {
        this.position = Cesium.Cartesian3.fromDegrees(lnglatArr[0], lnglatArr[1], lnglatArr[2] || 0);
      }

      this.entity = this.createGltfModel(this.position);
      callback(this.entity);
      this.state = "endCreate";
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (this.state == "startEdit" || this.state == "editing") return; //表示还没绘制完成

      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;
      var eidtModel;
      this.state = "startEdit";
      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          eidtModel = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (!eidtModel) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, [that.entity]);
        if (!cartesian) return;

        if (that.entity) {
          that.entity.position.setValue(cartesian);
          that.position = cartesian.clone();
        }

        that.state = "editing";
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!eidtModel) return;
        that.forbidDrawWorld(false);

        if (that.modifyHandler) {
          that.modifyHandler.destroy();
          that.modifyHandler = null;
        }

        that.state = "editing";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.state = "endCreate";

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.forbidDrawWorld(false);
      this.state = "endEdit";
    }
  }, {
    key: "createGltfModel",
    value: function createGltfModel(cartesian) {
      if (!cartesian) return;
      var heading = Cesium.Math.toRadians(this.style.heading);
      var pitch = Cesium.Math.toRadians(this.style.pitch);
      var roll = Cesium.Math.toRadians(this.style.roll);
      var hpr = new Cesium.HeadingPitchRoll(heading, pitch, roll);
      var orientation = Cesium.Transforms.headingPitchRollQuaternion(cartesian, hpr);
      var entity = this.viewer.entities.add({
        position: cartesian,
        orientation: orientation,
        model: {
          uri: this.modelUri,
          minimumPixelSize: this.style.minimumPixelSize,
          maximumScale: this.style.maximumScale,
          scale: this.style.scale || 1,
          heightReference: this.style.heightReference
        }
      });
      entity.objId = this.objId;
      return entity;
    }
  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      return isWgs84 ? util$1.cartesianToLnglat(this.position, this.viewer) : this.position;
    }
  }, {
    key: "getStyle",
    value: function getStyle() {
      var obj = {};
      var model = this.entity.model;
      obj.minimumPixelSize = model.minimumPixelSize.getValue();
      var orientation = this.entity.orientation.getValue();
      var p = this.entity.position.getValue(this.viewer.clock.currentTime);
      var hpr = util$1.oreatationToHpr(p.clone(), orientation, true) || {};
      obj.heading = (hpr.heading || 0) < 360 ? hpr.heading + 360 : hpr.heading;
      obj.pitch = hpr.pitch || 0;
      obj.roll = hpr.roll || 0;
      obj.scale = model.scale.getValue();
      obj.uri = model.uri.getValue();
      var heightReference = this.entity.heightReference && this.entity.heightReference.getValue();
      if (heightReference != undefined) obj.heightReference = Number(heightReference);
      return obj;
    }
  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      this.setOrientation(style.heading, style.pitch, style.roll);
      this.entity.model.scale.setValue(style.scale == undefined ? 1 : style.scale);
      if (style.uri) this.entity.model.uri.setValue(style.uri);
      if (style.heightReference != undefined) this.entity.model.heightReference.setValue(Number(style.heightReference));
      this.style = Object.assign(this.style, style);
    }
    /**
     * 设置模型姿态
     * @param {Number} h 偏转角
     * @param {Number} p 仰俯角
     * @param {Number} r 翻滚角
     */

  }, {
    key: "setOrientation",
    value: function setOrientation(h, p, r) {
      h = h || 0;
      p = p || 0;
      r = r || 0;
      this.style.heading = h;
      this.style.pitch = p;
      this.style.roll = r;
      var heading = Cesium.Math.toRadians(h || 0);
      var pitch = Cesium.Math.toRadians(p || 0);
      var roll = Cesium.Math.toRadians(r || 0);
      var hpr = new Cesium.HeadingPitchRoll(heading, pitch, roll);
      var position = this.entity.position._value;
      var orientation = Cesium.Transforms.headingPitchRollQuaternion(position, hpr);
      if (this.entity) this.entity.orientation = orientation;
    }
  }, {
    key: "remove",
    value: function remove() {
      if (this.entity) {
        this.state = "no";
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }
    }
  }, {
    key: "destroy",
    value: function destroy() {
      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      if (this.entity) {
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }

      this.style = null;

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }
    }
  }]);

  return CreateGltfModel;
}(BasePlot);

/**
 * 文字标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreateLabel
 */

var CreateLabel = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreateLabel, _BasePlot);

  var _super = _createSuper(CreateLabel);

  function CreateLabel(viewer, opt) {
    var _this;

    _classCallCheck(this, CreateLabel);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.type = "label";
    _this.viewer = viewer;
    /**
     * @property {Cesium.Cartesian3} 坐标
     */

    _this.position = null;
    return _this;
  }

  _createClass(CreateLabel, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      var that = this;
      this.state = "startCreate";
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;
        that.entity = that.createLabel(cartesian.clone());
        that.position = cartesian.clone();
        that.endCreate();
        if (callback) callback(that.entity);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.prompt.update(evt.endPosition, "单击新增");
        that.state = "creating";
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      if (!lnglatArr) return;
      this.state = "startCreate";
      var position = lnglatArr instanceof Cesium.Cartesian3 ? lnglatArr : Cesium.Cartesian3.fromDegrees(lnglatArr[0], lnglatArr[1], lnglatArr[2]);
      this.position = position;
      if (!position) return;
      this.entity = this.createLabel(position, this.style.text);
      if (callback) callback(this.entity);
      this.state = "endCreate";
    } // 设置相关样式

  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;

      if (style.fillColor) {
        var fillColor = style.fillColor instanceof Cesium.Color ? style.fillColor : Cesium.Color.fromCssColorString(style.fillColor || "#ffff00");
        fillColor = fillColor.withAlpha(style.fillColorAlpha || 1);
        this.entity.label.fillColor = fillColor;
      }

      this.entity.label.outlineWidth = style.outlineWidth;

      if (style.backgroundColor) {
        var backgroundColor = style.backgroundColor instanceof Cesium.Color ? style.backgroundColor : Cesium.Color.fromCssColorString(style.backgroundColor || "#000000");
        backgroundColor = backgroundColor.withAlpha(style.backgroundColorAlpha || 1);
        this.entity.label.backgroundColor = backgroundColor;
      }

      if (style.outlineColor) {
        var outlineColor = style.outlineColor instanceof Cesium.Color ? style.outlineColor : Cesium.Color.fromCssColorString(style.outlineColor || "#000000");
        outlineColor = outlineColor.withAlpha(style.outlineColorAlpha || 1);
        this.entity.label.outlineColor = outlineColor;
      }

      if (style.heightReference != undefined) this.entity.label.heightReference = Number(style.heightReference);
      if (style.pixelOffset) this.entity.label.pixelOffset = style.pixelOffset;
      if (style.text) this.entity.label.text = style.text;
      if (style.showBackground != undefined) this.entity.label.showBackground = Boolean(style.showBackground);

      if (style.scale) {
        this.entity.label.scale = Number(style.scale);
      }

      this.style = Object.assign(this.style, style);
    } // 获取相关样式

  }, {
    key: "getStyle",
    value: function getStyle() {
      var obj = {};
      var label = this.entity.label;
      var fillColor = label.fillColor.getValue();
      obj.fillColorAlpha = fillColor.alpha;
      obj.fillColor = new Cesium.Color(fillColor.red, fillColor.green, fillColor.blue, 1).toCssHexString();
      if (label.outlineWidth != undefined) obj.outlineWidth = label.outlineWidth._value;
      if (label.showBackground != undefined) obj.showBackground = Boolean(label.showBackground.getValue());

      if (label.backgroundColor) {
        var bkColor = label.backgroundColor.getValue();
        obj.backgroundColorAlpha = bkColor.alpha;
        obj.backgroundColor = new Cesium.Color(bkColor.red, bkColor.green, bkColor.blue, 1).toCssHexString();
      }

      if (label.outlineColor) {
        var outlineColor = label.outlineColor.getValue();
        obj.outlineColorAlpha = outlineColor.alpha;
        obj.outlineColor = new Cesium.Color(outlineColor.red, outlineColor.green, outlineColor.blue, 1).toCssHexString();
      }

      if (label.heightReference != undefined) {
        obj.heightReference = label.heightReference.getValue();
      }

      if (label.pixelOffset) obj.pixelOffset = label.pixelOffset.getValue();
      if (label.scale) obj.scale = label.scale.getValue();
      obj.text = label.text.getValue();
      return obj;
    }
  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      var lnglats = isWgs84 ? util$1.cartesianToLnglat(this.position) : this.position;
      return [lnglats]; // 包装一层 和其他标会类保持一致
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (this.state == "startEdit" || this.state == "editing" || !this.entity) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;
      var editLabel;
      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          editLabel = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (!editLabel) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (that.entity) {
          that.entity.position.setValue(cartesian);
          that.position = cartesian;
          that.state = "editing";
        }

        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!editLabel) return;
        that.forbidDrawWorld(false);

        if (that.modifyHandler) {
          that.modifyHandler.destroy();
          that.modifyHandler = null;
          that.state = "editing";
        }
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.forbidDrawWorld(false);
      this.state = "endEdit";
    }
  }, {
    key: "createLabel",
    value: function createLabel(cartesian) {
      if (!cartesian) return;
      var label = this.viewer.entities.add({
        position: cartesian,
        label: {
          text: this.style.text || "--",
          fillColor: this.style.fillColor ? Cesium.Color.fromCssColorString(this.style.fillColor).withAlpha(this.style.fillColorAlpha || 1) : Cesium.Color.WHITE,
          backgroundColor: this.style.backgroundColor ? Cesium.Color.fromCssColorString(this.style.backgroundColor).withAlpha(this.style.backgroundColorAlpha || 1) : Cesium.Color.WHITE,
          style: Cesium.LabelStyle.FILL,
          outlineWidth: this.style.outlineWidth || 4,
          scale: this.style.scale || 1,
          pixelOffset: this.style.pixelOffset || Cesium.Cartesian2.ZERO,
          showBackground: this.style.showBackground,
          heightReference: this.style.heightReference || 0,
          translucencyByDistance: new Cesium.NearFarScalar(10.0e6, 1.0, 20.0e6, 0.0), //sin_add 20241018
          // wzt 文字一致显示bug
          // disableDepthTestDistance: Number.MAX_VALUE
        }
      });
      label.objId = this.objId;
      return label;
    }
  }]);

  return CreateLabel;
}(BasePlot);

/**
 * 点标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreatePoint
 */

var CreatePoint = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreatePoint, _BasePlot);

  var _super = _createSuper(CreatePoint);

  function CreatePoint(viewer, opt) {
    var _this;

    _classCallCheck(this, CreatePoint);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.type = "point";
    _this.viewer = viewer;
    var defaultStyle = {
      color: Cesium.Color.AQUA,
      pixelSize: 10,
      outlineWidth: 1
    };
    _this.style = Object.assign(defaultStyle, _this.style || {});
    /**
     * @property {Cesium.Cartesian3} 坐标
     */

    _this.position = null;
    return _this;
  }

  _createClass(CreatePoint, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;
        that.entity = that.createPoint(cartesian);
        that.position = cartesian;

        if (that.handler) {
          that.handler.destroy();
          that.handler = null;
        }

        if (that.prompt) {
          that.prompt.destroy();
          that.prompt = null;
        }

        that.state = "endCreate";
        if (callback) callback(that.entity);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.prompt.update(evt.endPosition, "单击新增");
        that.state = "creating";
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      if (!lnglatArr) return;
      this.state = "startCreate";
      var position = lnglatArr instanceof Cesium.Cartesian3 ? lnglatArr : Cesium.Cartesian3.fromDegrees(lnglatArr[0], lnglatArr[1], lnglatArr[2]);
      this.position = position;
      if (!position) return;
      this.entity = this.createPoint(position);
      if (callback) callback(this.entity);
      this.state = "endCreate";
    } // 设置相关样式

  }, {
    key: "setStyle",
    value: function setStyle(style) {
      
      if (!style) return;

      if (style.color) {
        var color = Cesium.Color.fromCssColorString(style.color || "#ffff00");
        color = color.withAlpha(style.colorAlpha);
        this.entity.point.color = color;
      }
      this.entity.point.outlineWidth = Number(style.outlineWidth);

      if (style.outlineColor) {
        var outlineColor = Cesium.Color.fromCssColorString(style.outlineColor || "#000000");
        outlineColor = outlineColor.withAlpha(style.outlineColorAlpha);
        this.entity.point.outlineColor = outlineColor;
      }
      this.entity.point.heightReference = Number(style.heightReference);
      this.entity.point.pixelSize = Number(style.pixelSize);
      this.style = Object.assign(this.style, style);
    } // 获取相关样式

  }, {
    key: "getStyle",
    value: function getStyle() {
      var obj = {};
      var point = this.entity.point;
      var color = point.color.getValue();
      obj.colorAlpha = color.alpha;
      obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();
      obj.outlineWidth = point.outlineWidth._value;
      var outlineColor = point.outlineColor.getValue();
      obj.outlineColorAlpha = outlineColor.alpha;
      obj.outlineColor = new Cesium.Color(outlineColor.red, outlineColor.green, outlineColor.blue, 1).toCssHexString();
      if (point.heightReference != undefined) obj.heightReference = point.heightReference.getValue();
      obj.pixelSize = Number(point.pixelSize);
      return obj;
    }
  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      var lnglats = isWgs84 ? util$1.cartesianToLnglat(this.position) : this.position;
      return [lnglats]; // 包装一层 和其他标会类保持一致
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (this.state == "startEdit" || this.state == "editing" || !this.entity) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;
      var editPoint;
      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          editPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (!editPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (that.entity) {
          that.entity.position.setValue(cartesian);
          that.position = cartesian;
          that.state = "editing";
        }

        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!editPoint) return;
        that.forbidDrawWorld(false);

        if (that.modifyHandler) {
          that.modifyHandler.destroy();
          that.modifyHandler = null;
          that.state = "editing";
        }
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.forbidDrawWorld(false);
      this.state = "endEdit";
    }
  }, {
    key: "createPoint",
    value: function createPoint(cartesian) {
      if (!cartesian) return;
      var point = this.viewer.entities.add({
        position: cartesian,
        point: {
          color: this.style.color instanceof Cesium.Color ? this.style.color : this.style.color ? Cesium.Color.fromCssColorString(this.style.color).withAlpha(this.style.colorAlpha || 1) : Cesium.Color.WHITE,
          outlineColor: this.style.outlineColor instanceof Cesium.Color ? this.style.outlineColor : this.style.outlineColor ? Cesium.Color.fromCssColorString(this.style.outlineColor).withAlpha(this.style.outlineColorAlpha || 1) : Cesium.Color.BLACK,
          outlineWidth: this.style.outlineWidth || 4,
          pixelSize: this.style.pixelSize || 20,
          disableDepthTestDistance: Number.MAX_VALUE
        }
      });
      point.objId = this.objId;
      return point;
    }
  }]);

  return CreatePoint;
}(BasePlot);

/**
 * 面标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreatePolygon
 */

var CreatePolygon = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreatePolygon, _BasePlot);

  var _super = _createSuper(CreatePolygon);

  function CreatePolygon(viewer, opt) {
    var _this;

    _classCallCheck(this, CreatePolygon);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.type = "polygon";
    _this.viewer = viewer;
    _this.entity = null;
    _this.outline = null;
    var defaultStyle = {
      outlineColor: "#000000",
      outlineWidth: 2
    };
    _this.style = Object.assign(defaultStyle, _this.style || {});
    return _this;
  }

  _createClass(CreatePolygon, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer, []);
        if (!cartesian) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        that.positions.push(cartesian);
        var point = that.createPoint(cartesian);
        point.wz = that.positions.length - 1;
        that.controlPoints.push(point);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //移动时绘制面
        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          that.state = "startCreate";
          return;
        }

        if (that.prompt) that.prompt.update(evt.endPosition, "双击结束，右键取消上一步");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, []);
        if (!cartesian) return;
        if (that.positions.length >= 1) {
          that.state = "creating";

          if (!that.movePush) {
            that.positions.push(cartesian);
            that.movePush = true;
          } else {
            that.positions[that.positions.length - 1] = cartesian;
          }

          if (that.positions.length == 2) {
            if (!Cesium.defined(that.outline)) {
              that.outline = that.createPolyline(1);
            }
          }

          if (that.positions.length == 3) {
            if (!Cesium.defined(that.entity)) {
              that.entity = that.createPolygon(1);

              if (!that.style.outline && that.outline) {
                // 不需要创建轮廓 则后续删除
                that.outline.show = false;
              }

              that.entity.objId = that.objId;
            }
          }
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
        if (!that.entity) return;
        that.positions.splice(that.positions.length - 2, 1);
        that.viewer.entities.remove(that.controlPoints.pop());

        if (that.positions.length == 2) {
          if (that.entity) {
            that.viewer.entities.remove(that.entity);
            that.entity = null;
            if (that.outline) that.outline.show = true;
          }
        }

        if (that.positions.length == 1) {
          if (that.outline) {
            that.viewer.entities.remove(that.outline);
            that.outline = null;
          }

          if (that.prompt) that.prompt.update(evt.endPosition, "单击开始绘制");
          that.positions = [];
          that.movePush = false;
        }
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
      this.handler.setInputAction(function (evt) {
        //双击结束绘制
/*        if (!that.entity) return;
        that.endCreate();
        if (callback) callback(that.entity);*/
        //sin add 20240710
        that.movePush = false;
        that.endCreate();
        if (callback && that.entity) callback(that.entity);
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.state = "endCreate";
      that.positions.pop();
      that.viewer.entities.remove(that.controlPoints.pop());
      //sin add 20240710
      if(that.controlPoints.length < 3 && that.outline){
        that.viewer.entities.remove(that.outline);
      }

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      that.movePush = false;

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.viewer.trackedEntity = undefined;
      that.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      //通过传入坐标数组创建面
      if (!lnglatArr) return;
      this.state = "startCreate";
      var positions = lnglatArr[0] instanceof Cesium.Cartesian3 ? lnglatArr : util$1.lnglatsToCartesians(lnglatArr);
      if (!positions) return;
      this.positions = positions;
      this.entity = this.createPolygon();
      this.outline = this.createPolyline();
      this.outline.show = this.style.outline;
      
      for (var i = 0; i < positions.length; i++) {
        var newP = positions[i];
        var ctgc = Cesium.Cartographic.fromCartesian(positions[i]);
        var point = this.createPoint(newP);
        point.point.heightReference = this.style.heightReference;
        point.ctgc = ctgc;
        point.wz = this.controlPoints.length;
        this.controlPoints.push(point);
      }

      this.state = "endCreate";
      this.entity.objId = this.objId;
      if (callback) callback(this.entity);
    }
  }, {
    key: "getStyle",
    value: function getStyle() {
      if (!this.entity) return;
      var obj = {};
      var polygon = this.entity.polygon;

      if (polygon.material instanceof Cesium.ColorMaterialProperty) {
        obj.material = "common";
        var color = polygon.material.color.getValue();
        obj.colorAlpha = color.alpha;
        obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();
      }

      obj.fill = polygon.fill ? polygon.fill.getValue() : false;

      if (polygon.heightReference) {
        var heightReference = polygon.heightReference.getValue();
        obj.heightReference = Number(heightReference);
      }
      /* obj.heightReference = isNaN(polygon.heightReference.getValue()) ? false : polygon.heightReference.getValue(); */


      var outline = this.outline.polyline;

      if (outline && this.outline.show) {
        obj.outlineWidth = outline.width.getValue();
        /* obj.outline = "show"; */

        obj.outline = true;
        var oColor = outline.material.color.getValue();
        obj.outlineColorAlpha = oColor.alpha;
        obj.outlineColor = new Cesium.Color(oColor.red, oColor.green, oColor.blue, 1).toCssHexString();
      } else {
        /* obj.outline = "hide"; */
        obj.outline = false;
      }

      return obj;
    } // 设置相关样式

  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return; // 由于官方api中的outline限制太多 此处outline为重新构建的polyline

      /* this.outline.show = style.outline.show == "show" ? true : false; */

      this.outline.show = style.outline;
      if(this.outline.show ){
        var outline = this.outline.polyline;
        outline.width = style.outlineWidth;
        this.outline.clampToGround = Boolean(style.heightReference);
        var outlineColor = style.outlineColor instanceof Cesium.Color ? style.outlineColor : Cesium.Color.fromCssColorString(style.outlineColor);
        var outlineMaterial = outlineColor.withAlpha(style.outlineColorAlpha || 1);
        outline.material = outlineMaterial;
      }

      if (style.heightReference != undefined) this.entity.polygon.heightReference = Number(style.heightReference);
      var color = style.color instanceof Cesium.Color ? style.color : Cesium.Color.fromCssColorString(style.color);
      var material = color.withAlpha(style.colorAlpha || 1);
      this.entity.polygon.material = material;
      if (style.fill != undefined) this.entity.polygon.fill = style.fill;
      this.style = Object.assign(this.style, style);
    }
  }, 
  {
    key: "changeProperty",
    value: function changeProperty(code) {
      var that = this
      if(!that.entity) return
      if(code == 1) {
        let p1 = new Cesium.CallbackProperty(() => {
          return new Cesium.PolygonHierarchy(that.positions);
        }, false);
        let l1 = new Cesium.CallbackProperty(() => {
          return that.positions.concat([that.positions[0]]);
        }, false);
        that.entity._polygon.hierarchy = p1;
        that.outline._polyline.positions = l1;
      }else {
        that.entity._polygon.hierarchy = new Cesium.PolygonHierarchy(that.positions);
        that.outline._polyline.positions = that.positions.concat([that.positions[0]]);
      }
    }
  },
  {
    key: "createPolygon",
    value: function createPolygon(code) {
      var that = this;
      let hierarchy
      if(code) {
        hierarchy =  new Cesium.CallbackProperty(function () {
          return new Cesium.PolygonHierarchy(that.positions)
        }, false)
      }else {
        hierarchy = new Cesium.PolygonHierarchy(that.positions)
      }
      this.style.color = this.style.color || Cesium.Color.WHITE;
      this.style.outlineColor = this.style.outlineColor || Cesium.Color.BLACK;
      var polygonObj = {
        polygon: {
          hierarchy: hierarchy,
          heightReference: Number(this.style.heightReference),
          show: true,
          fill: this.style.fill == undefined ? true : this.style.fill,
          material: this.style.color instanceof Cesium.Color ? this.style.color : Cesium.Color.fromCssColorString(this.style.color).withAlpha(this.style.colorAlpha || 1)
        }
      };

      if (!this.style.heightReference) {
        polygonObj.polygon.height = 0; // 不贴地 必设

        polygonObj.polygon.perPositionHeight = true; // 启用点的真实高度
      }

      return this.viewer.entities.add(polygonObj);
    }
  }, {
    key: "createPolyline",
    value: function createPolyline(code) {
      var that = this;
      let newPositions
      if(code) {
        newPositions =  new Cesium.CallbackProperty(function () {
          return that.positions.concat(that.positions[0])
        }, false)
      }else {
        newPositions = that.positions.concat(that.positions[0])
      }
      var line = this.viewer.entities.add({
        polyline: {
          positions: newPositions,
          clampToGround: Boolean(this.style.heightReference),
          material: this.style.outlineColor instanceof Cesium.Color ? this.style.outlineColor : Cesium.Color.fromCssColorString(this.style.outlineColor).withAlpha(this.style.outlineColorAlpha || 1),
          width: this.style.outlineWidth || 1
        }
      });
      line.objId = this.objId;
      line.isOutline = true; // 标识其为边框线

      return line;
    }
  }, {
    key: "destroy",
    value: function destroy() {
      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      if (this.entity) {
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }

      if (this.outline) {
        this.viewer.entities.remove(this.outline);
        this.outline = null;
      }

      this.positions = [];
      this.style = null;

      if (this.modifyPoint) {
        this.viewer.entities.remove(this.modifyPoint);
        this.modifyPoint = null;
      }

      for (var i = 0; i < this.controlPoints.length; i++) {
        var point = this.controlPoints[i];
        this.viewer.entities.remove(point);
      }

      this.controlPoints = [];
      this.state = "no";
      if (this.prompt) this.prompt.destroy();

      if (this.outline) {
        this.outline = null;
        this.viewer.entities.remove(this.outline);
      }

      this.forbidDrawWorld(false);
    }
  }]);

  return CreatePolygon;
}(BasePlot);

/**
 * 矩形标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.BasePlot
 */

var CreateRectangle = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreateRectangle, _BasePlot);

  var _super = _createSuper(CreateRectangle);

  function CreateRectangle(viewer, opt) {
    var _this;

    _classCallCheck(this, CreateRectangle);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};
    _this.type = "rectangle";
    _this.viewer = viewer;
    /**
     * @property {Cesium.Entity} rightdownPoint 右下角实体点
     */

    _this.rightdownPoint = null;
    /**
    * @property {Cesium.Entity} leftupPoint 左上角实体点
    */

    _this.leftupPoint = null;
    /**
     * @property {Cesium.Cartesian3} leftup 左上角点坐标
     */

    _this.leftup = null;
    /**
    * @property {Cesium.Cartesian3} rightdown 右下角点坐标
    */

    _this.rightdown = null;

    _this.modifyPoint = null;
    _this.pointArr = [];
    _this.outline = undefined;
    return _this;
  }

  _createClass(CreateRectangle, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer, []);
        if (!cartesian) return;

        if (!that.leftupPoint) {
          that.leftup = cartesian;
          that.leftupPoint = that.createPoint(cartesian);
          that.leftupPoint.typeAttr = "leftup";
          that.rightdownPoint = that.createPoint(cartesian.clone());
          that.rightdown = cartesian.clone();
          that.rightdownPoint.typeAttr = "rightdown";
          that.entity = that.createRectangle(1);
          that.outline = that.createPolyline(1);
          that.outline.show = that.style.outline;
        } else {
          if (!that.entity) return;
          that.endCreate();
          if (callback) callback(that.entity);
        }

        console.log("单击开始创建");
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //移动时绘制线
        if (!that.leftupPoint) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          that.state = "startCreate";
          return;
        }

        that.prompt.update(evt.endPosition, "单击结束");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, []);
        if (!cartesian) return;

        if (that.rightdownPoint) {
          that.rightdownPoint.position.setValue(cartesian);
          that.rightdown = cartesian.clone();
          that.state = "creating";
        }
        console.log("单击开始创建时移动");
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.rightdownPoint) that.rightdownPoint.show = false;
      if (that.leftupPoint) that.leftupPoint.show = false;

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      
      if (this.state == "startEdit" || this.state == "editing" || !this.entity) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;
      if (that.rightdownPoint) that.rightdownPoint.show = true;
      if (that.leftupPoint) that.leftupPoint.show = true;
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.entity) return;
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);

      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, [that.entity, that.modifyPoint]);

        if (!cartesian) {
          return;
        }

        that.state == "editing";

        if (that.modifyPoint.typeAttr == "leftup") {
          that.leftup = cartesian;
          that.leftupPoint.position.setValue(that.leftup);
          // that.entity.position.setValue(that.leftup);
        } else {
          that.rightdown = cartesian;
          that.rightdownPoint.position.setValue(that.rightdown);
        }
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.forbidDrawWorld(false);
        that.state == "editing";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit(callback) {
      if (this.rightdownPoint) this.rightdownPoint.show = false;
      if (this.leftupPoint) this.leftupPoint.show = false;

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
        if (callback) callback(this.entity);
      }

      this.forbidDrawWorld(false);
      this.state = "endEdit";
    }
  },  {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      //通过传入坐标数组创建面
      if (!lnglatArr) return;
      this.state = "startCreate";
      var positions = lnglatArr[0] instanceof Cesium.Cartesian3 ? lnglatArr : util$1.lnglatsToCartesians(lnglatArr);
      if (!positions) return;
      this.leftup = positions[0];
      this.leftupPoint = this.createPoint(positions[0]);
      this.leftupPoint.typeAttr = "leftup";
      this.rightdownPoint = this.createPoint(positions[1].clone());
      this.rightdown = positions[1].clone();
      this.rightdownPoint.typeAttr = "rightdown";
      this.positions = positions;
      this.entity = this.createRectangle();
      this.outline = this.createPolyline();
      this.outline.show = this.style.outline;

      for (var i = 0; i < positions.length; i++) {
        var newP = positions[i];
        var ctgc = Cesium.Cartographic.fromCartesian(positions[i]);
        var point = this.createPoint(newP);
        point.point.heightReference = this.style.heightReference;
        point.ctgc = ctgc;
        point.wz = this.controlPoints.length;
        this.controlPoints.push(point);
      }

      this.state = "endCreate";
      this.entity.objId = this.objId;
      if (callback) callback(this.entity);
    }
  }, 
  {
    key: "changeProperty",
    value: function changeProperty(code) {
      var that = this
      if(!that.entity) return
      if(code) {
        let rC = new Cesium.CallbackProperty(function () {
          return Cesium.Rectangle.fromCartesianArray([that.leftup, that.rightdown]);
        }, false);
        let l1 = new Cesium.CallbackProperty(function () {
          var ctgc_leftup = Cesium.Cartographic.fromCartesian(that.leftup);
          var ctgc_rightdown = Cesium.Cartographic.fromCartesian(that.rightdown);
          var p1 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_leftup.latitude);
          var p2 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_rightdown.latitude);
          var p3 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_rightdown.latitude);
          var p4 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_leftup.latitude);
          return [p1, p2, p3, p4, p1];
        }, false)
        that.entity.rectangle.coordinates = rC
        that.outline._polyline.positions = l1;
      }else {
        var ctgc_leftup = Cesium.Cartographic.fromCartesian(that.leftup);
        var ctgc_rightdown = Cesium.Cartographic.fromCartesian(that.rightdown);
        var p1 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_leftup.latitude);
        var p2 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_rightdown.latitude);
        var p3 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_rightdown.latitude);
        var p4 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_leftup.latitude);
        let lP = [p1, p2, p3, p4, p1]
        that.entity.rectangle.coordinates = Cesium.Rectangle.fromCartesianArray([that.leftup, that.rightdown]);
        that.outline._polyline.positions = lP;
      }
    }
  },{
    key: "createRectangle",
    value: function createRectangle(code) {
      var that = this;
      let coordinates
      if(code) {
        coordinates = new Cesium.CallbackProperty(function () {
          return Cesium.Rectangle.fromCartesianArray([that.leftup, that.rightdown]);
        }, false)
      }else {
        coordinates = Cesium.Rectangle.fromCartesianArray([that.leftup, that.rightdown]);
      }
      var rectangle = this.viewer.entities.add({
        rectangle: {
          coordinates: coordinates,
          heightReference: this.style.heightReference || 0,
          show: true,
          fill: this.style.fill == undefined ? true : this.style.fill,
          material: this.style.color instanceof Cesium.Color ? this.style.color : this.style.color ? Cesium.Color.fromCssColorString(this.style.color).withAlpha(this.style.colorAlpha || 1) : Cesium.Color.WHITE
        }
      });
      rectangle.objId = this.objId;
      return rectangle;
    }
  }, {
    key: "createPolyline",
    value: function createPolyline(code) {
      var that = this;
      let lP
      if(code) {
        lP = new Cesium.CallbackProperty(function () {
          var ctgc_leftup = Cesium.Cartographic.fromCartesian(that.leftup);
          var ctgc_rightdown = Cesium.Cartographic.fromCartesian(that.rightdown);
          var p1 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_leftup.latitude);
          var p2 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_rightdown.latitude);
          var p3 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_rightdown.latitude);
          var p4 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_leftup.latitude);
          return [p1, p2, p3, p4, p1];
        }, false)
      }else {
        var ctgc_leftup = Cesium.Cartographic.fromCartesian(that.leftup);
          var ctgc_rightdown = Cesium.Cartographic.fromCartesian(that.rightdown);
          var p1 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_leftup.latitude);
          var p2 = Cesium.Cartesian3.fromRadians(ctgc_leftup.longitude, ctgc_rightdown.latitude);
          var p3 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_rightdown.latitude);
          var p4 = Cesium.Cartesian3.fromRadians(ctgc_rightdown.longitude, ctgc_leftup.latitude);
        lP = [p1, p2, p3, p4, p1]
      }
      
      var line = this.viewer.entities.add({
        polyline: {
          positions: lP,
          clampToGround: Boolean(this.style.heightReference),
          arcType:Cesium.ArcType.RHUMB,
          material: this.style.outlineColor instanceof Cesium.Color ? this.style.outlineColor : Cesium.Color.fromCssColorString(this.style.outlineColor).withAlpha(this.style.outlineColorAlpha || 1),
          width: this.style.outlineWidth || 1
        }
      });
      line.objId = this.objId;
      line.isOutline = true; // 标识其为边框线

      return line;
    }
  }, {
    key: "getPositions",
    value: function getPositions(isWgs84) {
      var positions = [];

      if (isWgs84) {
        positions = util$1.cartesiansToLnglats([this.leftup, this.rightdown], this.viewer);
      } else {
        positions = [this.leftup, this.rightdown];
      }

      return positions;
    }
  }, {
    key: "getStyle",
    value: function getStyle() {
      var obj = {};
      var rectangle = this.entity.rectangle; // 获取材质

      if (rectangle.material instanceof Cesium.ColorMaterialProperty) {
        // 颜色材质
        var color = rectangle.material.color.getValue();
        obj.colorAlpha = color.alpha;
        obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();
      } // 边框线


      var polyline = this.outline.polyline;
      obj.outline = this.outline.show;

      if (polyline) {
        obj.outlineWidth = polyline.width.getValue();
        var outlineColor = polyline.material.color.getValue();
        obj.outlineColorAlpha = outlineColor.alpha;
        obj.outlineColor = new Cesium.Color(outlineColor.red, outlineColor.green, outlineColor.blue, 1).toCssHexString();
      }

      if (obj.height) obj.height = rectangle.height.getValue();
      if (rectangle.fill) obj.fill = rectangle.fill.getValue();
      obj.heightReference = rectangle.heightReference.getValue();
      if (obj.heightReference == 1) obj.height = undefined;
      return obj;
    }
  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      console.log("rectangle setStyle====>", style);
      var color = style.color instanceof Cesium.Color ? style.color : Cesium.Color.fromCssColorString(style.color || "#ffff00");
      if (style.colorAlpha) color = color.withAlpha(style.colorAlpha);
      this.entity.rectangle.material = color; // 设置边框线

      this.outline.show = style.outline;
      this.outline.polyline.width = style.outlineWidth || 1.0;
      var outlineColor = style.outlineColor instanceof Cesium.Color ? style.outlineColor : Cesium.Color.fromCssColorString(style.outlineColor || "#000000");
      outlineColor = outlineColor.withAlpha(style.outlineColorAlpha || 1);
      this.outline.polyline.material = outlineColor;
      this.outline.polyline.clampToGround = Number(style.heightReference) == 1 ? true : false;
      this.entity.rectangle.heightReference = Number(style.heightReference);
      this.entity.rectangle.fill = style.fill;
      this.style = Object.assign(this.style, style);
    }
    /**
       * 销毁
       */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      if (this.entity) {
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }

      if (this.outline) {
        this.viewer.entities.remove(this.outline);
        this.outline = null;
      }

      if (this.leftupPoint) {
        this.viewer.entities.remove(this.leftupPoint);
        this.leftupPoint = null;
      }

      if (this.rightdownPoint) {
        this.viewer.entities.remove(this.rightdownPoint);
        this.rightdownPoint = null;
      }

      this.positions = [];
      this.style = null;
      this.modifyPoint = null;

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      this.state = "no";
      this.forbidDrawWorld(false);
    }
  }]);

  return CreateRectangle;
}(BasePlot);

/**
 * 流动线材质
 * @constructor
 * @param {Object} opt 基础配置
 * @param {Cesium.Color} opt.color 颜色
 * @param {Number} [opt.duration=1000] 时间间隔（ms）
 * @param {String} opt.image 材质图片
 * @param {Cesium.Cartesian2} [opt.repeat=new Cesium.Cartesian2(1.0, 1.0)] 平铺
 * @example
 * var line3 = viewer.entities.add({
    name: "飞行弧线",
    polyline: {
      positions: points4,
      width: 15,
      material: new FlowLineMaterial({
        image: "../img/texture/rightarrow.png",
        repeat: new Cesium.Cartesian2(100, 1),
        duration: 2500
      })
    }
  });
 */
function FlowLineMaterial(opt) {
  this.defaultColor = new Cesium.Color(0, 0, 0, 0);
  opt = opt || {};
  this._definitionChanged = new Cesium.Event();
  this._color = undefined;
  this.color = opt.color || this.defaultColor; //颜色

  this._duration = opt.duration || 1000; //时长

  this.url = opt.image; //材质图片

  this._time = undefined;
  this.repeat = opt.repeat || new Cesium.Cartesian2(1.0, 1.0);
}

FlowLineMaterial.prototype.getType = function (time) {
  return "FlowLine";
};

FlowLineMaterial.prototype.getValue = function (time, result) {
  if (!Cesium.defined(result)) {
    result = {};
  }

  result.color = Cesium.Property.getValueOrClonedDefault(this.color, time, this.defaultColor, result.color);
  result.image = this.url;

  if (this._time === undefined) {
    this._time = new Date().getTime();
  }

  result.time = (new Date().getTime() - this._time) / this._duration;
  result.repeat = this.repeat;
  return result;
};

FlowLineMaterial.prototype.equals = function (other) {
  return this === other || other instanceof FlowLineMaterial && Cesium.Property.equals(this._color, other._color) && this._image._value == other._image._value && this.repeat.equals(other.repeat);
};

FlowLineMaterial.prototype.isConstant = function () {
  return false;
};

FlowLineMaterial.prototype.definitionChanged = function () {
  return this._definitionChanged;
};

Object.defineProperties(FlowLineMaterial.prototype, {
  isConstant: {
    get: function get() {
      return false;
    }
  },
  definitionChanged: {
    get: function get() {
      return this._definitionChanged;
    }
  },
  color: Cesium.createPropertyDescriptor('color')
});

Cesium.Material._materialCache.addMaterial("FlowLine", {
  fabric: {
    type: "FlowLine",
    uniforms: {
      color: new Cesium.Color(1, 0, 0, 1.0),
      image: '',
      time: 0,
      repeat: new Cesium.Cartesian2(1.0, 1.0)
    },
    source: "czm_material czm_getMaterial(czm_materialInput materialInput)\n\
            {\n\
                czm_material material = czm_getDefaultMaterial(materialInput);\n\
                vec2 st = repeat * materialInput.st;\n\
                vec4 colorImage = texture(image, vec2(fract(st.s - time), st.t));\n\
                if(color.a == 0.0)\n\
                {\n\
                    material.alpha = colorImage.a;\n\
                    material.diffuse = colorImage.rgb; \n\
                }\n\
                else\n\
                {\n\
                    material.alpha = colorImage.a * color.a;\n\
                    material.diffuse = max(color.rgb * material.alpha * 3.0, color.rgb); \n\
                }\n\
                return material;\n\
            }"
  },
  translucent: function translucent() {
    return true;
  }
});

/**
 * 墙体材质
 * @constructor
 * @param {Object} opt 基础配置
 * @param {Cesium.Color} opt.color 颜色
 * @param {Number} [opt.duration=1000] 时间间隔（ms）
 * @param {Cesium.Cartesian2} [opt.repeat=new Cesium.Cartesian2(5, 1)] 平铺
 * @param {Boolean} [axisY=false] 方向轴是否为y轴
 * @param {String} opt.image 材质图片
 * @example
 * viewer.entities.add({
    name: '动态立体墙',
    wall: {
      positions: positions,
      maximumHeights: maximumHeights,
      minimumHeights: minimumHeights,
      material: new WallMaterial({
        color: Cesium.Color.RED,
        duration: 3000,
        axisY: false,
        image: "../img/texture/glow.png",
        repeat: new Cesium.Cartesian2(1, 1) //平铺
      })
    }
  });
 */
function WallMaterial(opt) {
  this._definitionChanged = new Cesium.Event();
  this._color = opt.color;
  this.duration = opt.duration || 1000;
  this._time = new Date().getTime();

  if (!opt.image) {
    console.log("未传入材料图片！");
  }

  this.image = opt.image;
  this.repeat = opt.repeat || new Cesium.Cartesian2(5, 1);
  this.axisY = opt.axisY;
}

WallMaterial.prototype.getType = function (time) {
  return 'WallMaterial';
};

WallMaterial.prototype.getValue = function (time, result) {
  if (!Cesium.defined(result)) {
    result = {};
  }

  result.color = this._color || Cesium.Color.WHITE;
  result.image = this.image;
  result.time = (new Date().getTime() - this._time) % this.duration / this.duration;
  result.axisY = this.axisY;
  result.repeat = this.repeat;
  return result;
};

WallMaterial.prototype.equals = function (other) {
  return this === other || other instanceof WallMaterial && Cesium.Property.equals(this._color, other._color) && this._image._value == other._image._value && this.repeat.equals(other.repeat);
};

Object.defineProperties(WallMaterial.prototype, {
  isConstant: {
    get: function get() {
      return false;
    }
  },
  definitionChanged: {
    get: function get() {
      return this._definitionChanged;
    }
  },
  color: Cesium.createPropertyDescriptor('color')
});

Cesium.Material._materialCache.addMaterial('WallMaterial', {
  fabric: {
    type: 'WallMaterial',
    uniforms: {
      color: new Cesium.Color(1.0, 0.0, 0.0, 0.5),
      image: "",
      time: 0,
      repeat: new Cesium.Cartesian2(5, 1),
      axisY: false
    },
    source: "czm_material czm_getMaterial(czm_materialInput materialInput)\n\
                      {\n\
                          czm_material material = czm_getDefaultMaterial(materialInput);\n\
                          vec2 st = repeat * materialInput.st;\n\
                          vec4 colorImage = texture(image, vec2(fract((axisY?st.s:st.t) - time), st.t));\n\
                          if(color.a == 0.0)\n\
                          {\n\
                              material.alpha = colorImage.a;\n\
                              material.diffuse = colorImage.rgb; \n\
                          }\n\
                          else\n\
                          {\n\
                              material.alpha = colorImage.a * color.a;\n\
                              material.diffuse = max(color.rgb * material.alpha * 3.0, color.rgb); \n\
                          }\n\
                          // material.emission = colorImage.rgb;\n\
                          return material;\n\
                      }"
  },
  translucent: function translucent(material) {
    return true;
  }
});

/**
 * 波纹材质
 * @constructor
 * @param {Object} opt 基础配置
 * @param {Cesium.Color} opt.color 颜色
 * @param {Number} [opt.duration=1000] 时间间隔（ms）
 * @example
 * var redEllipse = viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(103.0, 40.0),
    ellipse: {
      semiMinorAxis: 250000.0,
      semiMajorAxis: 400000.0,
      material: new WaveMaterial({
        duration: 2000,
        color: Cesium.Color.RED,
      }),
    },
  });
 */
function WaveMaterial(opt) {
  this._definitionChanged = new Cesium.Event();
  this._color = undefined;
  this.defaultColor = Cesium.Color.fromCssColorString("#02ff00");
  this.color = Cesium.defaultValue(opt.color, this.defaultColor); //颜色

  this._duration = opt.duration || 1000; //时长

  this._time = undefined;
}

WaveMaterial.prototype.color = function () {
  return Cesium.createPropertyDescriptor('color');
};

WaveMaterial.prototype.getType = function () {
  return 'WaveMaterial';
};

WaveMaterial.prototype.getValue = function (time, result) {
  if (!Cesium.defined(result)) {
    result = {};
  }

  result.color = Cesium.Property.getValueOrClonedDefault(this._color, time, this.color, result.color);

  if (this._time === undefined) {
    this._time = new Date().getTime();
  }

  result.time = (new Date().getTime() - this._time) / this._duration;
  return result;
};

WaveMaterial.prototype.equals = function (other) {
  return this === other || other instanceof WaveMaterial && Cesium.Property.equals(this._color, other._color);
};

Object.defineProperties(WaveMaterial.prototype, {
  isConstant: {
    get: function get() {
      return false;
    }
  },
  definitionChanged: {
    get: function get() {
      return this._definitionChanged;
    }
  }
});

Cesium.Material._materialCache.addMaterial("WaveMaterial", {
  fabric: {
    type: "WaveMaterial",
    uniforms: {
      color: new Cesium.Color(1, 0, 0, 1.0),
      time: 10
    },
    source: "czm_material czm_getMaterial(czm_materialInput materialInput)\n            {\n                czm_material material = czm_getDefaultMaterial(materialInput);\n                material.diffuse = 1.5 * color.rgb;\n                vec2 st = materialInput.st;\n                float dis = distance(st, vec2(0.5, 0.5));\n                float per = fract(time);\n                if(dis > per * 0.5){\n                    discard;\n                }else {\n                    material.alpha = color.a  * dis / per / 2.0;\n                }\n                return material;\n            }\n        "
  },
  translucent: function translucent() {
    return true;
  }
});

/**
 * 动态扫描
 * @param {Object} options 
 * @param {Cesium.Color} options.color  锥体颜色
 * @param {Number} options.duration  扫描速度
 */
function ScanMaterial(opt) {
  this.defaultColor = new Cesium.Color(0, 0, 0, 0);
  opt = opt || {};
  this._definitionChanged = new Cesium.Event();
  this._color = undefined;
  this.color = opt.color || this.defaultColor; //颜色

  this._duration = opt.duration || 1000; //时长

  this._time = undefined;
}

ScanMaterial.prototype.getType = function (time) {
  return "ScanMaterial";
};

ScanMaterial.prototype.getValue = function (time, result) {
  if (!Cesium.defined(result)) {
    result = {};
  }

  result.color = Cesium.Property.getValueOrClonedDefault(this.color, time, this.defaultColor, result.color);

  if (this._time === undefined) {
    this._time = new Date().getTime();
  }

  result.time = (new Date().getTime() - this._time) / this._duration;
  return result;
};

ScanMaterial.prototype.equals = function (other) {
  return this === other || other instanceof ScanMaterial && Cesium.Property.equals(this._color, other._color);
};

Object.defineProperties(ScanMaterial.prototype, {
  isConstant: {
    get: function get() {
      return false;
    }
  },
  definitionChanged: {
    get: function get() {
      return this._definitionChanged;
    }
  },
  color: Cesium.createPropertyDescriptor('color')
});

Cesium.Material._materialCache.addMaterial("ScanMaterial", {
  fabric: {
    type: "",
    uniforms: {
      color: new Cesium.Color(1, 0, 0, 1.0),
      time: new Date().getTime(),
      corver: 90,
      speed: 5
    },
    source: "czm_material czm_getMaterial(czm_materialInput materialInput){\n                    czm_material material = czm_getDefaultMaterial(materialInput);\n                    vec2 st = materialInput.st;\n                    st.x = st.x - 0.5;\n                    st.y = st.y - 0.5;\n                    vec2 normalize_st = normalize(st);\n                    float rotateAngle = mod(time * speed,360.0);\n                    vec2 center_y = vec2(1.0,0.0);\n                    center_y.x = cos(rotateAngle);\n                    center_y.y = sin(rotateAngle);\n\n                    vec2 normalize_center_y = normalize(center_y);\n                    // \u8BA1\u7B97\u5F53\u524D\u7EB9\u7406\u5750\u6807\u548C\u4E2D\u5FC3\u70B9\u7684\u5939\u89D2\n                    float angle_cos_y = dot(normalize_center_y,normalize_st);\n                    angle_cos_y = acos(angle_cos_y);\n                    float angle = degrees(angle_cos_y);\n\n                    vec3 normalize_center_y_vec3 = vec3(normalize_center_y,0.0);\n                    vec3 st_vec3 = vec3(st,0.0);\n                    vec3 cross_value = cross(normalize_center_y_vec3,st_vec3);\n                    if(cross_value.z > 0.0){\n                        angle = angle + 360.0;\n                    }\n\n                    float alpha ;\n                    if(angle > corver){\n                        alpha = 0.0;\n                    }else{\n                        alpha = 1.0 - angle/corver;\n                    }\n\n                    material.diffuse = color.rgb;\n                    material.alpha = alpha;\n                    return material;\n        }"
  },
  translucent: function translucent() {
    return true;
  }
});

/**
 * 动态锥体
 * @param {Object} options 
 * @param {Cesium.Color} options.color  锥体颜色
 * @param {Number} options.duration  速度
 * @param {Boolean} options.bottom  是否显示锥体底部
 */
function CylinderMaterial(options) {
  options = Cesium.defaultValue(options, Cesium.defaultValue.EMPTY_OBJECT);
  this._definitionChanged = new Cesium.Event();
  this._color = undefined;
  this._colorSubscription = undefined;
  this.color = Cesium.defaultValue(options.color, Cesium.Color.WHITE); //颜色

  this._duration = options.duration || 1000; //时长

  this.bottom = options.bottom == undefined ? true : options.bottom;
  this._time = undefined;
}

CylinderMaterial.prototype.getType = function (time) {
  return 'cylinderMaterial';
};

CylinderMaterial.prototype.getValue = function (time, result) {
  if (!Cesium.defined(result)) {
    result = {};
  }

  result.color = Cesium.Property.getValueOrClonedDefault(this._color, time, Cesium.Color.WHITE, result.color);
  result.bottom = this.bottom;

  if (this._time === undefined) {
    this._time = new Date().getTime();
  }

  result.time = (new Date().getTime() - this._time) / this._duration;
  return result;
};

CylinderMaterial.prototype.equals = function (other) {
  return this === other || other instanceof CylinderMaterial && Property.equals(this._color, other._color);
};

Object.defineProperties(CylinderMaterial.prototype, {
  isConstant: {
    get: function get() {
      return false;
    }
  },
  definitionChanged: {
    get: function get() {
      return this._definitionChanged;
    }
  },
  color: Cesium.createPropertyDescriptor('color')
});

Cesium.Material._materialCache.addMaterial('cylinderMaterial', {
  fabric: {
    type: Cesium.Material.CircleFadeMaterialType,
    uniforms: {
      color: new Cesium.Color(1, 0, 0, 1.0),
      time: 1,
      bottom: true
    },
    source: "czm_material czm_getMaterial(czm_materialInput materialInput)\n\t    {\n            czm_material material = czm_getDefaultMaterial(materialInput);\n            material.diffuse = 1.5 * color.rgb;\n            vec2 st = materialInput.st;\n            float dis = distance(st, vec2(0.5, 0.5));\n            float per = fract(time);\n            if(dis > per * 0.5){\n                //material.alpha = 0.0;\n                discard;\n            }else {\n                material.alpha = color.a  * dis / per ;\n            }\n\n            if(!bottom){\n                vec3 v_normalMC = czm_inverseNormal * materialInput.normalEC;\n                vec3 axis_z = vec3(0.0, 0.0, 1.0);\n                if (dot(axis_z, v_normalMC) > 0.95){\n                    material.alpha = 0.0;\n                }\n            }\n           \n            return material;\n\t    }"
  },
  translucent: function translucent() {
    return true;
  }
});

var EllipsoidTrailMaterial = /*#__PURE__*/function () {
  function EllipsoidTrailMaterial(options) {
    _classCallCheck(this, EllipsoidTrailMaterial);

    this._definitionChanged = new Cesium.Event();
    this._color = undefined;
    this._speed = undefined;
    this.color = options.color;
    this.speed = options.speed;
  }

  _createClass(EllipsoidTrailMaterial, [{
    key: "isConstant",
    get: function get() {
      return false;
    }
  }, {
    key: "definitionChanged",
    get: function get() {
      return this._definitionChanged;
    }
  }, {
    key: "getType",
    value: function getType(time) {
      return "ellipsoidTrailMaterial";
    }
  }, {
    key: "getValue",
    value: function getValue(time, result) {
      if (!Cesium.defined(result)) {
        result = {};
      }

      result.color = Cesium.Property.getValueOrDefault(this._color, time, Cesium.Color.RED, result.color);
      result.speed = Cesium.Property.getValueOrDefault(this._speed, time, 10, result.speed);
      return result;
    }
  }, {
    key: "equals",
    value: function equals(other) {
      return this === other || other instanceof EllipsoidTrailMaterial && Cesium.Property.equals(this._color, other._color) && Cesium.Property.equals(this._speed, other._speed);
    }
  }]);

  return EllipsoidTrailMaterial;
}();

Object.defineProperties(EllipsoidTrailMaterial.prototype, {
  color: Cesium.createPropertyDescriptor('color'),
  speed: Cesium.createPropertyDescriptor('speed')
});

Cesium.Material._materialCache.addMaterial("ellipsoidTrailMaterial", {
  fabric: {
    type: "ellipsoidTrailMaterial",
    uniforms: {
      color: new Cesium.Color(1.0, 0.0, 0.0, 1.0),
      speed: 2
    },
    source: "\n                uniform vec4 color;\n                uniform float speed;\n                czm_material czm_getMaterial(czm_materialInput materialInput){\n                czm_material material = czm_getDefaultMaterial(materialInput);\n                vec2 st = materialInput.st;\n                float time = fract(czm_frameNumber * speed / 1000.0);\n                float alpha = abs(smoothstep(0.5,1.,fract( -st.t - time)));\n                alpha += .1;\n                material.alpha = alpha;\n                material.diffuse = color.rgb;\n                return material;\n            }\n    "
  },
  translucent: function translucent(material) {
    return true;
  }
});

var material = {
  // FlowLine: FlowLineMaterial,
  // Wall: WallMaterial,
  // Wave: WaveMaterial,
  // Scan: ScanMaterial,
  // Cylinder: CylinderMaterial,
  // Elipsoid: EllipsoidTrailMaterial,
  LineFlow: FlowLineMaterial,
  // 动态线
  Wall: WallMaterial,
  EllipseWave: WaveMaterial,
  // 扩散圆
  EllipseScan: ScanMaterial,
  // 扫描圆
  CylinderScan: CylinderMaterial,
  // 锥体扫描
  Elipsoid: EllipsoidTrailMaterial // 动态球体

};

/**
 * 线标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreatePolyline
 */

var CreatePolyline = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreatePolyline, _BasePlot);

  var _super = _createSuper(CreatePolyline);

  function CreatePolyline(viewer, opt) {
    var _this;

    _classCallCheck(this, CreatePolyline);

    _this = _super.call(this, viewer, opt);
    _this.movePush = false;
    _this.type = "polyline";
    /**
     * @property {Number} [maxPointNum=Number.MAX_VALUE] 线的最大点位数量
    */

    _this.maxPointNum = _this.style.maxPointNum || Number.MAX_VALUE; // 最多点数

    return _this;
  }
  /**
   * 开始绘制
   * @param {Function} callback 绘制完成之后的回调函数
   */


  _createClass(CreatePolyline, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer, [that.entity]);
        if (!cartesian) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        that.positions.push(cartesian);
        var point = that.createPoint(cartesian);
        point.wz = that.positions.length - 1;
        that.controlPoints.push(point); // 达到最大数量 结束绘制

        if (that.positions.length == that.maxPointNum) {
          that.endCreate();
          if (callback) callback(that.entity);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //移动时绘制线
        that.state = "creating";

        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          that.state = "startCreate";
          return;
        }

        that.prompt.update(evt.endPosition, "右键取消上一步，双击结束");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer, [that.entity]);
        if (!cartesian) return;

        if (!that.movePush) {
          that.positions.push(cartesian);
          that.movePush = true;
        } else {
          that.positions[that.positions.length - 1] = cartesian;
        }

        if (that.positions.length == 2) {
          if (!Cesium.defined(that.entity)) {
            that.entity = that.createPolyline(1);
          }
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
        //右键取消上一步
        if (!that.entity) {
          return;
        }

        that.positions.splice(that.positions.length - 2, 1);
        that.viewer.entities.remove(that.controlPoints.pop());

        if (that.positions.length == 1) {
          if (that.entity) {
            that.viewer.entities.remove(that.entity);
            that.entity = null;
          }

          that.prompt.update(evt.endPosition, "单击开始绘制");
          that.movePush = false;
          that.positions = [];
        }
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
      this.handler.setInputAction(function (evt) {
        //双击结束绘制
        if (!that.entity) {
          return;
        }

        that.endCreate();
        if (callback) callback(that.entity);
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.state = "endCreate";

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      that.positions.pop();
      that.viewer.entities.remove(that.controlPoints.pop());

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.viewer.trackedEntity = undefined;
      that.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      //通过传入坐标数组创建面
      if (!lnglatArr || lnglatArr.length < 1) return;
      this.state = "startCreate";
      var positions = lnglatArr[0] instanceof Cesium.Cartesian3 ? lnglatArr : util$1.lnglatsToCartesians(lnglatArr);
      if (!positions) return;
      this.positions = positions;
      this.entity = this.createPolyline();
      if (callback) callback(this.entity);

      for (var i = 0; i < positions.length; i++) {
        var newP = positions[i];
        var point = this.createPoint(newP);

        if (this.style.clampToGround) {
          point.point.heightReference = 1;
        }

        point.wz = this.controlPoints.length;
        this.controlPoints.push(point);
      }

      this.state = "endCreate";
    }
  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      var material = this.getMaterial(style.material, style);
      this.entity.polyline.material = material;
      this.entity.polyline.clampToGround = Boolean(style.clampToGround);
      if (style.width) this.entity.polyline.width = style.width || 3;
      this.style = Object.assign(this.style, style);
    } // 获取相关样式

  }, {
    key: "getStyle",
    value: function getStyle() {
      if (!this.entity) return;
      var obj = {};
      var polyline = this.entity.polyline;

      if (this.style.animateType != undefined) {
        obj.animateType = this.style.animateType;
        obj.image = this.style.image;
        obj.duration = this.style.duration;
      }

      if (polyline.material instanceof Cesium.ColorMaterialProperty) {
        obj.material = "common";
      } else {
        obj.material = "flowline";
        obj.duration = polyline.material._duration;
        obj.image = polyline.material.url;
      }

      var color = polyline.material.color.getValue();
      obj.colorAlpha = color.alpha;
      obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();
      obj.width = polyline.width._value;
      var clampToGround = polyline.clampToGround ? polyline.clampToGround.getValue() : false;
      obj.clampToGround = Boolean(clampToGround);
      return obj;
    }
  }, {
    key: "changeProperty",
    value: function changeProperty(code) {
      var that = this
      if(!that.entity) return
      if(code == 1) {
        let p1 = new Cesium.CallbackProperty(() => {
          return that.positions;
        }, false);
        that.entity._polyline.positions = p1
      }else {
        that.entity._polyline.positions = that.positions
      }
    }
  }, {
    key: "createPolyline",
    value: function createPolyline(code) {
      var that = this;
      let lP
      if(code) {
        lP = new Cesium.CallbackProperty(function () {
          return that.positions;
        }, false)
      }else {
        lP = that.positions;
      }
      var polyline = this.viewer.entities.add({
        polyline: {
          positions: lP,
          show: true,
          material: this.getMaterial(this.style.animateType, this.style),
          width: this.style.width || 3,
          clampToGround: this.style.clampToGround
        }
      });
      polyline.objId = this.objId; // 此处进行和entityObj进行关联

      return polyline;
    }
  }, {
    key: "getMaterial",
    value: function getMaterial(animateType, style) {
      // 构建多种材质的线
      style = style || {};
      var material$1 = null;
      var color = style.color || Cesium.Color.WHITE;
      color = color instanceof Cesium.Color ? color : Cesium.Color.fromCssColorString(style.color);
      color = color.withAlpha(style.colorAlpha || 1);

      if (animateType == "flowline") {
        if (!style.image) {
          console.log("动态材质，缺少纹理图片");
          return color;
        }

        material$1 = new material.LineFlow({
          color: color,
          // 默认颜色
          image: style.image,
          duration: style.duration || 5000
        });
      } else if (animateType == "flyline") {
        if (!style.image) {
          console.log("动态材质，缺少纹理图片");
          return color;
        }

        material$1 = new material.LineFlow({
          //动画线材质
          color: color,
          duration: style.duration || 3000,
          image: style.image,
          repeat: new Cesium.Cartesian2(1, 1) //平铺

        });
      } else {
        material$1 = color;
      }

      return material$1;
    }
  }]);

  return CreatePolyline;
}(BasePlot);

// 箭头的基本计算方法
var ArrowUtil = /*#__PURE__*/function () {
  function ArrowUtil(opt) {
    _classCallCheck(this, ArrowUtil);

    this.FITTING_COUNT = 100;
    this.HALF_PI = Math.PI / 2;
    this.ZERO_TOLERANCE = 0.0001;
    this.TWO_PI = Math.PI * 2;
    this.headHeightFactor = opt.headHeightFactor;
    this.headWidthFactor = opt.headWidthFactor;
    this.neckHeightFactor = opt.neckHeightFactor;
    this.neckWidthFactor = opt.neckWidthFactor;
    this.headTailFactor = opt.headTailFactor;
    this.tailWidthFactor = opt.tailWidthFactor;
    this.swallowTailFactor = opt.swallowTailFactor;
  } //空间坐标转投影坐标


  _createClass(ArrowUtil, [{
    key: "cartesian32WeMercator",
    value: function cartesian32WeMercator(position) {
      if (!position) return;
      var lnglat = this.cartesianToLnglat(position);
      return this.lnglat2WeMercator(lnglat);
    } //投影坐标转空间坐标

  }, {
    key: "webMercator2Cartesian3",
    value: function webMercator2Cartesian3(arg) {
      if (!arg) return;
      var lnglat = this.webMercator2Lnglat(arg);
      return Cesium.Cartesian3.fromDegrees(lnglat[0], lnglat[1]);
    } //投影坐标转地理坐标

  }, {
    key: "webMercator2Lnglat",
    value: function webMercator2Lnglat(points) {
      if (!points) return;
      var x = points[0] / 20037508.34 * 180;
      var y = points[1] / 20037508.34 * 180;
      y = 180 / Math.PI * (2 * Math.atan(Math.exp(y * Math.PI / 180)) - Math.PI / 2);
      return [x, y];
    } //地理坐标转投影坐标

  }, {
    key: "lnglat2WeMercator",
    value: function lnglat2WeMercator(lnglat) {
      if (!lnglat) return;
      var x = lnglat[0] * 20037508.34 / 180;
      var y = Math.log(Math.tan((90 + lnglat[1]) * Math.PI / 360)) / (Math.PI / 180);
      y = y * 20037508.34 / 180;
      return [x, y];
    } //获取第三点 

  }, {
    key: "getThirdPoint",
    value: function getThirdPoint(startPnt, endPnt, angle, distance, clockWise) {
      var azimuth = this.getAzimuth(startPnt, endPnt);
      var alpha = clockWise ? azimuth + angle : azimuth - angle;
      var dx = distance * Math.cos(alpha);
      var dy = distance * Math.sin(alpha);
      return [endPnt[0] + dx, endPnt[1] + dy];
    } //计算夹角

  }, {
    key: "getAzimuth",
    value: function getAzimuth(startPoint, endPoint) {
      var azimuth = void 0;
      var angle = Math.asin(Math.abs(endPoint[1] - startPoint[1]) / this.MathDistance(startPoint, endPoint));

      if (endPoint[1] >= startPoint[1] && endPoint[0] >= startPoint[0]) {
        azimuth = angle + Math.PI;
      } else if (endPoint[1] >= startPoint[1] && endPoint[0] < startPoint[0]) {
        azimuth = Math.PI * 2 - angle;
      } else if (endPoint[1] < startPoint[1] && endPoint[0] < startPoint[0]) {
        azimuth = angle;
      } else if (endPoint[1] < startPoint[1] && endPoint[0] >= startPoint[0]) {
        azimuth = Math.PI - angle;
      }

      return azimuth;
    }
  }, {
    key: "MathDistance",
    value: function MathDistance(pnt1, pnt2) {
      var a = Math.pow(pnt1[0] - pnt2[0], 2);
      var b = Math.pow(pnt1[1] - pnt2[1], 2);
      var c = Math.sqrt(a + b) || 0.001; // 防止做分母  导致报错

      return c;
    } //计算闭合曲面上的点

  }, {
    key: "isClockWise",
    value: function isClockWise(pnt1, pnt2, pnt3) {
      return (pnt3[1] - pnt1[1]) * (pnt2[0] - pnt1[0]) > (pnt2[1] - pnt1[1]) * (pnt3[0] - pnt1[0]);
    }
    /* getBisectorNormals(t, pnt1, pnt2, pnt3) {
      var normal = this.getNormal(pnt1, pnt2, pnt3);
      var bisectorNormalRight = null,
        bisectorNormalLeft = null,
        dt = null,
        x = null,
        y = null;
        var dist = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
      var uX = normal[0] / dist;
      var uY = normal[1] / dist;
      var d1 = this.MathDistance(pnt1, pnt2);
      var d2 = this.MathDistance(pnt2, pnt3);
      if (dist > this.ZERO_TOLERANCE) {
        if (this.isClockWise(pnt1, pnt2, pnt3)) {
          dt = t * d1;
          x = pnt2[0] - dt * uY;
          y = pnt2[1] + dt * uX;
          bisectorNormalRight = [x, y];
          dt = t * d2;
          x = pnt2[0] + dt * uY;
          y = pnt2[1] - dt * uX;
          bisectorNormalLeft = [x, y];
        } else {
          dt = t * d1;
          x = pnt2[0] + dt * uY;
          y = pnt2[1] - dt * uX;
          bisectorNormalRight = [x, y];
          dt = t * d2;
          x = pnt2[0] - dt * uY;
          y = pnt2[1] + dt * uX;
          bisectorNormalLeft = [x, y];
        }
      } else {
        x = pnt2[0] + t * (pnt1[0] - pnt2[0]);
        y = pnt2[1] + t * (pnt1[1] - pnt2[1]);
        bisectorNormalRight = [x, y];
        x = pnt2[0] + t * (pnt3[0] - pnt2[0]);
        y = pnt2[1] + t * (pnt3[1] - pnt2[1]);
        bisectorNormalLeft = [x, y];
      }
      return [bisectorNormalRight, bisectorNormalLeft];
    } */

    /* getCubicValue (t, startPnt, cPnt1, cPnt2, endPnt) {
      t = Math.max(Math.min(t, 1), 0);
      var tp = 1 - t,
        t2 = t * t;
        var t3 = t2 * t;
      var tp2 = tp * tp;
      var tp3 = tp2 * tp;
      var x = tp3 * startPnt[0] + 3 * tp2 * t * cPnt1[0] + 3 * tp * t2 * cPnt2[0] + t3 * endPnt[0];
      var y = tp3 * startPnt[1] + 3 * tp2 * t * cPnt1[1] + 3 * tp * t2 * cPnt2[1] + t3 * endPnt[1];
      return [x, y];
    } */

  }, {
    key: "getNormal",
    value: function getNormal(pnt1, pnt2, pnt3) {
      var dX1 = pnt1[0] - pnt2[0];
      var dY1 = pnt1[1] - pnt2[1];
      var d1 = Math.sqrt(dX1 * dX1 + dY1 * dY1);
      dX1 /= d1;
      dY1 /= d1;
      var dX2 = pnt3[0] - pnt2[0];
      var dY2 = pnt3[1] - pnt2[1];
      var d2 = Math.sqrt(dX2 * dX2 + dY2 * dY2);
      dX2 /= d2;
      dY2 /= d2;
      var uX = dX1 + dX2;
      var uY = dY1 + dY2;
      return [uX, uY];
    }
  }, {
    key: "getArcPoints",
    value: function getArcPoints(center, radius, startAngle, endAngle) {
      var x = null,
          y = null,
          pnts = [],
          angleDiff = endAngle - startAngle;
      angleDiff = angleDiff < 0 ? angleDiff + Math.PI * 2 : angleDiff;

      for (var i = 0; i <= 100; i++) {
        var angle = startAngle + angleDiff * i / 100;
        x = center[0] + radius * Math.cos(angle);
        y = center[1] + radius * Math.sin(angle);
        pnts.push([x, y]);
      }

      return pnts;
    }
  }, {
    key: "getBaseLength",
    value: function getBaseLength(points) {
      return Math.pow(this.wholeDistance(points), 0.99);
    }
  }, {
    key: "wholeDistance",
    value: function wholeDistance(points) {
      var distance = 0;
      var that = this;

      if (points && Array.isArray(points) && points.length > 0) {
        points.forEach(function (item, index) {
          if (index < points.length - 1) {
            distance += that.MathDistance(item, points[index + 1]);
          }
        });
      }

      return distance;
    } // getArrowHeadPoints(obj) {
    //     if (!obj) return [];
    //     var points = obj.points;
    //     var tailLeft = obj.tailLeft;
    //     var tailRight = obj.tailRight;
    //     var headTailFactor = obj.headTailFactor;
    //     var neckWidthFactor = obj.neckWidthFactor;
    //     var neckHeightFactor = obj.neckHeightFactor;
    //     var headWidthFactor = obj.headWidthFactor;
    //     var headHeightFactor = obj.headHeightFactor;
    //     var len = this.getBaseLength(points);
    //     var headHeight = len * headHeightFactor;
    //     var headPnt = points[points.length - 1];
    //     len = this.MathDistance(headPnt, points[points.length - 2]);
    //     var tailWidth = this.MathDistance(tailLeft, tailRight);
    //     if (headHeight > tailWidth * headTailFactor) {
    //         headHeight = tailWidth * headTailFactor;
    //     }
    //     var headWidth = headHeight * headWidthFactor;
    //     var neckWidth = headHeight * neckWidthFactor;
    //     headHeight = headHeight > len ? len : headHeight;
    //     var neckHeight = headHeight * neckHeightFactor;
    //     var headEndPnt = this.getThirdPoint(points[points.length - 2], headPnt, 0, headHeight, true);
    //     var neckEndPnt = this.getThirdPoint(points[points.length - 2], headPnt, 0, neckHeight, true);
    //     var headLeft = this.getThirdPoint(headPnt, headEndPnt, this.HALF_PI, headWidth, false);
    //     var headRight = this.getThirdPoint(headPnt, headEndPnt, this.HALF_PI, headWidth, true);
    //     var neckLeft = this.getThirdPoint(headPnt, neckEndPnt, this.HALF_PI, neckWidth, false);
    //     var neckRight = this.getThirdPoint(headPnt, neckEndPnt, this.HALF_PI, neckWidth, true);
    //     return [neckLeft, headLeft, headPnt, headRight, neckRight];
    // }

  }, {
    key: "getArrowHeadPoints",
    value: function getArrowHeadPoints(points, tailLeft, tailRight) {
      this.DGIndex = points.length;
      this.points = points;
      var len = this.getBaseLength(points);
      var headHeight = len * this.headHeightFactor;
      var headPnt = points[points.length - 1];
      len = this.MathDistance(headPnt, points[points.length - 2]);
      var tailWidth = this.MathDistance(tailLeft, tailRight);

      if (headHeight > tailWidth * this.headTailFactor) {
        headHeight = tailWidth * this.headTailFactor;
      }

      var headWidth = headHeight * this.headWidthFactor;
      var neckWidth = headHeight * this.neckWidthFactor;
      headHeight = headHeight > len ? len : headHeight;
      var neckHeight = headHeight * this.neckHeightFactor;
      var headEndPnt = this.getThirdPoint(points[points.length - 2], headPnt, 0, headHeight, true);
      var neckEndPnt = this.getThirdPoint(points[points.length - 2], headPnt, 0, neckHeight, true);
      var headLeft = this.getThirdPoint(headPnt, headEndPnt, Math.PI / 2, headWidth, false);
      var headRight = this.getThirdPoint(headPnt, headEndPnt, Math.PI / 2, headWidth, true);
      var neckLeft = this.getThirdPoint(headPnt, neckEndPnt, Math.PI / 2, neckWidth, false);
      var neckRight = this.getThirdPoint(headPnt, neckEndPnt, Math.PI / 2, neckWidth, true);
      return [neckLeft, headLeft, headPnt, headRight, neckRight];
    }
  }, {
    key: "getArrowHeadPointsNoLR",
    value: function getArrowHeadPointsNoLR(points) {
      var len = this.getBaseLength(points);
      var headHeight = len * this.headHeightFactor;
      var headPnt = points[points.length - 1];
      var headWidth = headHeight * this.headWidthFactor;
      var neckWidth = headHeight * this.neckWidthFactor;
      var neckHeight = headHeight * this.neckHeightFactor;
      var headEndPnt = this.getThirdPoint(points[points.length - 2], headPnt, 0, headHeight, true);
      var neckEndPnt = this.getThirdPoint(points[points.length - 2], headPnt, 0, neckHeight, true);
      var headLeft = this.getThirdPoint(headPnt, headEndPnt, Math.PI / 2, headWidth, false);
      var headRight = this.getThirdPoint(headPnt, headEndPnt, Math.PI / 2, headWidth, true);
      var neckLeft = this.getThirdPoint(headPnt, neckEndPnt, Math.PI / 2, neckWidth, false);
      var neckRight = this.getThirdPoint(headPnt, neckEndPnt, Math.PI / 2, neckWidth, true);
      return [neckLeft, headLeft, headPnt, headRight, neckRight];
    } // getTailPoints(points) {
    //     if (!points) return;
    //     var tailWidthFactor = this.tailWidthFactor;
    //     var swallowTailFactor = this.swallowTailFactor;
    //     var allLen = this.getBaseLength(points);
    //     var tailWidth = allLen * tailWidthFactor;
    //     var tailLeft = this.getThirdPoint(points[1], points[0], this.HALF_PI, tailWidth, false);
    //     var tailRight = this.getThirdPoint(points[1], points[0], this.HALF_PI, tailWidth, true);
    //     var len = tailWidth * swallowTailFactor;
    //     var swallowTailPnt = this.getThirdPoint(points[1], points[0], 0, len, true);
    //     return [tailLeft, swallowTailPnt, tailRight];
    // }

  }, {
    key: "getTailPoints",
    value: function getTailPoints(points) {
      var allLen = this.getBaseLength(points);
      var tailWidth = allLen * this.tailWidthFactor;
      var tailLeft = this.getThirdPoint(points[1], points[0], Math.PI / 2, tailWidth, false);
      var tailRight = this.getThirdPoint(points[1], points[0], Math.PI / 2, tailWidth, true);
      return [tailLeft, tailRight];
    }
  }, {
    key: "getArrowBodyPoints",
    value: function getArrowBodyPoints(points, neckLeft, neckRight, tailWidthFactor) {
      var allLen = this.wholeDistance(points);
      var len = this.getBaseLength(points);
      var tailWidth = len * tailWidthFactor;
      var neckWidth = this.MathDistance(neckLeft, neckRight);
      var widthDif = (tailWidth - neckWidth) / 2;
      var tempLen = 0,
          leftBodyPnts = [],
          rightBodyPnts = [];

      for (var i = 1; i < points.length - 1; i++) {
        var angle = this.getAngleOfThreePoints(points[i - 1], points[i], points[i + 1]) / 2;
        tempLen += this.MathDistance(points[i - 1], points[i]);
        var w = (tailWidth / 2 - tempLen / allLen * widthDif) / Math.sin(angle);
        var left = this.getThirdPoint(points[i - 1], points[i], Math.PI - angle, w, true);
        var right = this.getThirdPoint(points[i - 1], points[i], angle, w, false);
        leftBodyPnts.push(left);
        rightBodyPnts.push(right);
      }

      return leftBodyPnts.concat(rightBodyPnts);
    }
  }, {
    key: "getAngleOfThreePoints",
    value: function getAngleOfThreePoints(pntA, pntB, pntC) {
      var angle = this.getAzimuth(pntB, pntA) - this.getAzimuth(pntB, pntC);
      return angle < 0 ? angle + Math.PI * 2 : angle;
    }
  }, {
    key: "getQBSplinePoints",
    value: function getQBSplinePoints(points) {
      if (points.length <= 2) {
        return points;
      } else {
        var n = 2,
            bSplinePoints = [];
        var m = points.length - n - 1;
        bSplinePoints.push(points[0]);

        for (var i = 0; i <= m; i++) {
          for (var t = 0; t <= 1; t += 0.05) {
            var x = 0,
                y = 0;

            for (var k = 0; k <= n; k++) {
              var factor = this.getQuadricBSplineFactor(k, t);
              x += factor * points[i + k][0];
              y += factor * points[i + k][1];
            }

            bSplinePoints.push([x, y]);
          }
        }

        bSplinePoints.push(points[points.length - 1]);
        return bSplinePoints;
      }
    }
  }, {
    key: "getQuadricBSplineFactor",
    value: function getQuadricBSplineFactor(k, t) {
      var res = 0;

      if (k === 0) {
        res = Math.pow(t - 1, 2) / 2;
      } else if (k === 1) {
        res = (-2 * Math.pow(t, 2) + 2 * t + 1) / 2;
      } else if (k === 2) {
        res = Math.pow(t, 2) / 2;
      }

      return res;
    }
  }, {
    key: "Mid",
    value: function Mid(point1, point2) {
      return [(point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2];
    }
  }, {
    key: "getCircleCenterOfThreePoints",
    value: function getCircleCenterOfThreePoints(point1, point2, point3) {
      var pntA = [(point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2];
      var pntB = [pntA[0] - point1[1] + point2[1], pntA[1] + point1[0] - point2[0]];
      var pntC = [(point1[0] + point3[0]) / 2, (point1[1] + point3[1]) / 2];
      var pntD = [pntC[0] - point1[1] + point3[1], pntC[1] + point1[0] - point3[0]];
      return this.getIntersectPoint(pntA, pntB, pntC, pntD);
    }
  }, {
    key: "getIntersectPoint",
    value: function getIntersectPoint(pntA, pntB, pntC, pntD) {
      if (pntA[1] === pntB[1]) {
        var _f = (pntD[0] - pntC[0]) / (pntD[1] - pntC[1]);

        var _x = _f * (pntA[1] - pntC[1]) + pntC[0];

        var _y = pntA[1];
        return [_x, _y];
      }

      if (pntC[1] === pntD[1]) {
        var _e = (pntB[0] - pntA[0]) / (pntB[1] - pntA[1]);

        var _x2 = _e * (pntC[1] - pntA[1]) + pntA[0];

        var _y2 = pntC[1];
        return [_x2, _y2];
      }

      var e = (pntB[0] - pntA[0]) / (pntB[1] - pntA[1]);
      var f = (pntD[0] - pntC[0]) / (pntD[1] - pntC[1]);
      var y = (e * pntA[1] - pntA[0] - f * pntC[1] + pntC[0]) / (e - f);
      var x = e * y - e * pntA[1] + pntA[0];
      return [x, y];
    }
  }, {
    key: "getBezierPoints",
    value: function getBezierPoints(points) {
      if (points.length <= 2) {
        return points;
      } else {
        var bezierPoints = [];
        var n = points.length - 1;

        for (var t = 0; t <= 1; t += 0.01) {
          var x = 0,
              y = 0;

          for (var index = 0; index <= n; index++) {
            var factor = this.getBinomialFactor(n, index);
            var a = Math.pow(t, index);
            var b = Math.pow(1 - t, n - index);
            x += factor * a * b * points[index][0];
            y += factor * a * b * points[index][1];
          }

          bezierPoints.push([x, y]);
        }

        bezierPoints.push(points[n]);
        return bezierPoints;
      }
    }
  }, {
    key: "getFactorial",
    value: function getFactorial(n) {
      var result = 1;

      switch (n) {
        case n <= 1:
          result = 1;
          break;

        case n === 2:
          result = 2;
          break;

        case n === 3:
          result = 6;
          break;

        case n === 24:
          result = 24;
          break;

        case n === 5:
          result = 120;
          break;

        default:
          for (var i = 1; i <= n; i++) {
            result *= i;
          }

          break;
      }

      return result;
    }
  }, {
    key: "getBinomialFactor",
    value: function getBinomialFactor(n, index) {
      return this.getFactorial(n) / (this.getFactorial(index) * this.getFactorial(n - index));
    }
  }, {
    key: "cartesianToLnglat",
    value: function cartesianToLnglat(cartesian) {
      if (!cartesian) return;
      /* var ellipsoid = viewer.scene.globe.ellipsoid; */

      var lnglat = Cesium.Cartographic.fromCartesian(cartesian); // var lnglat = ellipsoid.cartesianToCartographic(cartesian);

      var lat = Cesium.Math.toDegrees(lnglat.latitude);
      var lng = Cesium.Math.toDegrees(lnglat.longitude);
      var hei = lnglat.height;
      return [lng, lat, hei];
    }
  }, {
    key: "getCurvePoints",
    value: function getCurvePoints(t, controlPoints) {
      var leftControl = this.getLeftMostControlPoint(controlPoints, t);
      var pnt1 = null,
          pnt2 = null,
          pnt3 = null,
          normals = [leftControl],
          points = [];

      for (var i = 0; i < controlPoints.length - 2; i++) {
        var _ref2 = [controlPoints[i], controlPoints[i + 1], controlPoints[i + 2]];
        pnt1 = _ref2[0];
        pnt2 = _ref2[1];
        pnt3 = _ref2[2];
        var normalPoints = this.getBisectorNormals(t, pnt1, pnt2, pnt3);
        normals = normals.concat(normalPoints);
      }

      var rightControl = this.getRightMostControlPoint(controlPoints, t);

      if (rightControl) {
        normals.push(rightControl);
      }

      for (var _i = 0; _i < controlPoints.length - 1; _i++) {
        pnt1 = controlPoints[_i];
        pnt2 = controlPoints[_i + 1];
        points.push(pnt1);

        for (var _t = 0; _t < this.FITTING_COUNT; _t++) {
          var pnt = this.getCubicValue(_t / this.FITTING_COUNT, pnt1, normals[_i * 2], normals[_i * 2 + 1], pnt2);
          points.push(pnt);
        }

        points.push(pnt2);
      }

      return points;
    }
  }, {
    key: "getCubicValue",
    value: function getCubicValue(t, startPnt, cPnt1, cPnt2, endPnt) {
      t = Math.max(Math.min(t, 1), 0);
      var tp = 1 - t,
          t2 = t * t;
      var t3 = t2 * t;
      var tp2 = tp * tp;
      var tp3 = tp2 * tp;
      var x = tp3 * startPnt[0] + 3 * tp2 * t * cPnt1[0] + 3 * tp * t2 * cPnt2[0] + t3 * endPnt[0];
      var y = tp3 * startPnt[1] + 3 * tp2 * t * cPnt1[1] + 3 * tp * t2 * cPnt2[1] + t3 * endPnt[1];
      return [x, y];
    }
  }, {
    key: "getLeftMostControlPoint",
    value: function getLeftMostControlPoint(controlPoints, t) {
      var _ref = [controlPoints[0], controlPoints[1], controlPoints[2], null, null],
          pnt1 = _ref[0],
          pnt2 = _ref[1],
          pnt3 = _ref[2],
          controlX = _ref[3],
          controlY = _ref[4];
      var pnts = this.getBisectorNormals(0, pnt1, pnt2, pnt3);
      var normalRight = pnts[0];
      var normal = this.getNormal(pnt1, pnt2, pnt3);
      var dist = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1]);

      if (dist > this.ZERO_TOLERANCE) {
        var mid = this.Mid(pnt1, pnt2);
        var pX = pnt1[0] - mid[0];
        var pY = pnt1[1] - mid[1];
        var d1 = this.MathDistance(pnt1, pnt2);
        var n = 2.0 / d1;
        var nX = -n * pY;
        var nY = n * pX;
        var a11 = nX * nX - nY * nY;
        var a12 = 2 * nX * nY;
        var a22 = nY * nY - nX * nX;
        var dX = normalRight[0] - mid[0];
        var dY = normalRight[1] - mid[1];
        controlX = mid[0] + a11 * dX + a12 * dY;
        controlY = mid[1] + a12 * dX + a22 * dY;
      } else {
        controlX = pnt1[0] + t * (pnt2[0] - pnt1[0]);
        controlY = pnt1[1] + t * (pnt2[1] - pnt1[1]);
      }

      return [controlX, controlY];
    }
  }, {
    key: "getBisectorNormals",
    value: function getBisectorNormals(t, pnt1, pnt2, pnt3) {
      var normal = this.getNormal(pnt1, pnt2, pnt3);
      var bisectorNormalRight = null,
          bisectorNormalLeft = null,
          dt = null,
          x = null,
          y = null;
      var dist = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
      var uX = normal[0] / dist;
      var uY = normal[1] / dist;
      var d1 = this.MathDistance(pnt1, pnt2);
      var d2 = this.MathDistance(pnt2, pnt3);

      if (dist > this.ZERO_TOLERANCE) {
        if (this.isClockWise(pnt1, pnt2, pnt3)) {
          dt = t * d1;
          x = pnt2[0] - dt * uY;
          y = pnt2[1] + dt * uX;
          bisectorNormalRight = [x, y];
          dt = t * d2;
          x = pnt2[0] + dt * uY;
          y = pnt2[1] - dt * uX;
          bisectorNormalLeft = [x, y];
        } else {
          dt = t * d1;
          x = pnt2[0] + dt * uY;
          y = pnt2[1] - dt * uX;
          bisectorNormalRight = [x, y];
          dt = t * d2;
          x = pnt2[0] - dt * uY;
          y = pnt2[1] + dt * uX;
          bisectorNormalLeft = [x, y];
        }
      } else {
        x = pnt2[0] + t * (pnt1[0] - pnt2[0]);
        y = pnt2[1] + t * (pnt1[1] - pnt2[1]);
        bisectorNormalRight = [x, y];
        x = pnt2[0] + t * (pnt3[0] - pnt2[0]);
        y = pnt2[1] + t * (pnt3[1] - pnt2[1]);
        bisectorNormalLeft = [x, y];
      }

      return [bisectorNormalRight, bisectorNormalLeft];
    }
  }, {
    key: "getRightMostControlPoint",
    value: function getRightMostControlPoint(controlPoints, t) {
      var count = controlPoints.length;
      var pnt1 = controlPoints[count - 3];
      var pnt2 = controlPoints[count - 2];
      var pnt3 = controlPoints[count - 1];
      var pnts = this.getBisectorNormals(0, pnt1, pnt2, pnt3);
      var normalLeft = pnts[1];
      var normal = this.getNormal(pnt1, pnt2, pnt3);
      var dist = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
      var controlX = null,
          controlY = null;

      if (dist > this.ZERO_TOLERANCE) {
        var mid = this.Mid(pnt2, pnt3);
        var pX = pnt3[0] - mid[0];
        var pY = pnt3[1] - mid[1];
        var d1 = this.MathDistance(pnt2, pnt3);
        var n = 2.0 / d1;
        var nX = -n * pY;
        var nY = n * pX;
        var a11 = nX * nX - nY * nY;
        var a12 = 2 * nX * nY;
        var a22 = nY * nY - nX * nX;
        var dX = normalLeft[0] - mid[0];
        var dY = normalLeft[1] - mid[1];
        controlX = mid[0] + a11 * dX + a12 * dY;
        controlY = mid[1] + a12 * dX + a22 * dY;
      } else {
        controlX = pnt3[0] + t * (pnt2[0] - pnt3[0]);
        controlY = pnt3[1] + t * (pnt2[1] - pnt3[1]);
      }

      return [controlX, controlY];
    }
  }]);

  return ArrowUtil;
}();

var AttackArrow = /*#__PURE__*/function () {
  function AttackArrow(opt) {
    _classCallCheck(this, AttackArrow);

    this.type = "AttackArrow";
    if (!opt) opt = {}; //影响因素

    opt.headHeightFactor = opt.headHeightFactor || 0.18;
    opt.headWidthFactor = opt.headWidthFactor || 0.3;
    opt.neckHeightFactor = opt.neckHeightFactor || 0.85;
    opt.neckWidthFactor = opt.neckWidthFactor || 0.15;
    opt.headTailFactor = opt.headTailFactor || 0.8;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(AttackArrow, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (!positions) return;
      this.positions = positions;
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1]],
          tailLeft = _ref[0],
          tailRight = _ref[1];

      if (this.plotUtil.isClockWise(pnts[0], pnts[1], pnts[2])) {
        tailLeft = pnts[1];
        tailRight = pnts[0];
      }

      var midTail = this.plotUtil.Mid(tailLeft, tailRight);
      var bonePnts = [midTail].concat(pnts.slice(2));
      var headPnts = this.plotUtil.getArrowHeadPoints(bonePnts, tailLeft, tailRight);

      if (!headPnts || headPnts.length == 0) {
        console.warn("计算面数据有误，不计算，返回传入坐标数组！");
        return positions;
      }

      var _ref2 = [headPnts[0], headPnts[4]],
          neckLeft = _ref2[0],
          neckRight = _ref2[1];
      var tailWidthFactor = this.plotUtil.MathDistance(tailLeft, tailRight) / this.plotUtil.getBaseLength(bonePnts);
      var bodyPnts = this.plotUtil.getArrowBodyPoints(bonePnts, neckLeft, neckRight, tailWidthFactor);
      var count = bodyPnts.length;
      var leftPnts = [tailLeft].concat(bodyPnts.slice(0, count / 2));
      leftPnts.push(neckLeft);
      var rightPnts = [tailRight].concat(bodyPnts.slice(count / 2, count));
      rightPnts.push(neckRight);
      leftPnts = this.plotUtil.getQBSplinePoints(leftPnts);
      rightPnts = this.plotUtil.getQBSplinePoints(rightPnts);
      var pList = leftPnts.concat(headPnts, rightPnts.reverse());
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return AttackArrow;
}();

var AttackArrowPW = /*#__PURE__*/function () {
  function AttackArrowPW(arg) {
    _classCallCheck(this, AttackArrowPW);

    if (!arg) arg = {}; //影响因素

    var opt = {};
    opt.headHeightFactor = arg.headHeightFactor || 0.18;
    opt.headWidthFactor = arg.headWidthFactor || 0.3;
    opt.neckHeightFactor = arg.neckHeightFactor || 0.85;
    opt.neckWidthFactor = arg.neckWidthFactor || 0.15;
    opt.tailWidthFactor = this.tailWidthFactor = arg.tailWidthFactor || 0.1;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(AttackArrowPW, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (!positions) return;
      this.positions = positions;
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var tailPnts = this.plotUtil.getTailPoints(pnts);
      var headPnts = this.plotUtil.getArrowHeadPoints(pnts, tailPnts[0], tailPnts[1]);
      var neckLeft = headPnts[0];
      var neckRight = headPnts[4];
      var bodyPnts = this.plotUtil.getArrowBodyPoints(pnts, neckLeft, neckRight, this.tailWidthFactor);
      var _count = bodyPnts.length;
      var leftPnts = [tailPnts[0]].concat(bodyPnts.slice(0, _count / 2));
      leftPnts.push(neckLeft);
      var rightPnts = [tailPnts[1]].concat(bodyPnts.slice(_count / 2, _count));
      rightPnts.push(neckRight);
      leftPnts = this.plotUtil.getQBSplinePoints(leftPnts);
      rightPnts = this.plotUtil.getQBSplinePoints(rightPnts);
      var pList = leftPnts.concat(headPnts, rightPnts.reverse());
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return AttackArrowPW;
}();

var AttackArrowYW = /*#__PURE__*/function () {
  function AttackArrowYW(arg) {
    _classCallCheck(this, AttackArrowYW);

    if (!arg) arg = {};
    var opt = {}; //影响因素

    opt.headHeightFactor = arg.headHeightFactor || 0.18;
    opt.headWidthFactor = arg.headWidthFactor || 0.3;
    opt.neckHeightFactor = arg.neckHeightFactor || 0.85;
    opt.neckWidthFactor = arg.neckWidthFactor || 0.15;
    opt.tailWidthFactor = this.tailWidthFactor = arg.tailWidthFactor || 0.1;
    opt.headTailFactor = arg.headTailFactor || 0.8;
    opt.swallowTailFactor = this.swallowTailFactor = arg.swallowTailFactor || 1;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(AttackArrowYW, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (!positions) return;
      this.positions = positions;
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1]],
          tailLeft = _ref[0],
          tailRight = _ref[1];

      if (this.plotUtil.isClockWise(pnts[0], pnts[1], pnts[2])) {
        tailLeft = pnts[1];
        tailRight = pnts[0];
      }

      var midTail = this.plotUtil.Mid(tailLeft, tailRight);
      var bonePnts = [midTail].concat(pnts.slice(2));
      var headPnts = this.plotUtil.getArrowHeadPoints(bonePnts, tailLeft, tailRight);
      var _ref2 = [headPnts[0], headPnts[4]],
          neckLeft = _ref2[0],
          neckRight = _ref2[1];
      var tailWidth = this.plotUtil.MathDistance(tailLeft, tailRight);
      var allLen = this.plotUtil.getBaseLength(bonePnts);
      var len = allLen * this.tailWidthFactor * this.swallowTailFactor;
      var swallowTailPnt = this.plotUtil.getThirdPoint(bonePnts[1], bonePnts[0], 0, len, true);
      var factor = tailWidth / allLen;
      var bodyPnts = this.plotUtil.getArrowBodyPoints(bonePnts, neckLeft, neckRight, factor);
      var count = bodyPnts.length;
      var leftPnts = [tailLeft].concat(bodyPnts.slice(0, count / 2));
      leftPnts.push(neckLeft);
      var rightPnts = [tailRight].concat(bodyPnts.slice(count / 2, count));
      rightPnts.push(neckRight);
      leftPnts = this.plotUtil.getQBSplinePoints(leftPnts);
      rightPnts = this.plotUtil.getQBSplinePoints(rightPnts);
      var pList = leftPnts.concat(headPnts, rightPnts.reverse(), [swallowTailPnt, leftPnts[0]]);
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return AttackArrowYW;
}();

var CloseCurve = /*#__PURE__*/function () {
  function CloseCurve(arg) {
    _classCallCheck(this, CloseCurve);

    var opt = {}; //影响因素

    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(CloseCurve, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      pnts.push(pnts[0], pnts[1]);
      var normals = [];
      var pList = [];

      for (var i = 0; i < pnts.length - 2; i++) {
        var normalPoints = this.plotUtil.getBisectorNormals(0.3, pnts[i], pnts[i + 1], pnts[i + 2]);
        normals = normals.concat(normalPoints);
      }

      var count = normals.length;
      normals = [normals[count - 1]].concat(normals.slice(0, count - 1));

      for (var _i = 0; _i < pnts.length - 2; _i++) {
        var pnt1 = pnts[_i];
        var pnt2 = pnts[_i + 1];
        pList.push(pnt1);

        for (var t = 0; t <= 100; t++) {
          var pnt = this.plotUtil.getCubicValue(t / 100, pnt1, normals[_i * 2], normals[_i * 2 + 1], pnt2);
          pList.push(pnt);
        }

        pList.push(pnt2);
      }

      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return CloseCurve;
}();

var Curve = /*#__PURE__*/function () {
  function Curve(arg) {
    _classCallCheck(this, Curve);

    var opt = {}; //影响因素

    this.typeName = "Curve";
    this.plotUtil = new ArrowUtil(opt);
    this.t = 0.3;
  }

  _createClass(Curve, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var pList = [];

      if (pnts.length < 2) {
        return false;
      } else if (pnts.length === 2) {
        pList = pnts;
      } else {
        pList = this.plotUtil.getCurvePoints(this.t, pnts);
      }

      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return Curve;
}();

var CurveFlag = /*#__PURE__*/function () {
  function CurveFlag(arg) {
    _classCallCheck(this, CurveFlag);

    var opt = {}; //影响因素

    this.typeName = "CurveFlag";
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(CurveFlag, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var pList = [];

      if (pnts.length > 1) {
        var startPoint = pnts[0];
        var endPoint = pnts[pnts.length - 1];
        var point1 = startPoint;
        var point2 = [(endPoint[0] - startPoint[0]) / 4 + startPoint[0], (endPoint[1] - startPoint[1]) / 8 + startPoint[1]];
        var point3 = [(startPoint[0] + endPoint[0]) / 2, startPoint[1]];
        var point4 = [(endPoint[0] - startPoint[0]) * 3 / 4 + startPoint[0], -(endPoint[1] - startPoint[1]) / 8 + startPoint[1]];
        var point5 = [endPoint[0], startPoint[1]];
        var point6 = [endPoint[0], (startPoint[1] + endPoint[1]) / 2];
        var point7 = [(endPoint[0] - startPoint[0]) * 3 / 4 + startPoint[0], (endPoint[1] - startPoint[1]) * 3 / 8 + startPoint[1]];
        var point8 = [(startPoint[0] + endPoint[0]) / 2, (startPoint[1] + endPoint[1]) / 2];
        var point9 = [(endPoint[0] - startPoint[0]) / 4 + startPoint[0], (endPoint[1] - startPoint[1]) * 5 / 8 + startPoint[1]];
        var point10 = [startPoint[0], (startPoint[1] + endPoint[1]) / 2];
        var point11 = [startPoint[0], endPoint[1]];
        var curve1 = this.plotUtil.getBezierPoints([point1, point2, point3, point4, point5]);
        var curve2 = this.plotUtil.getBezierPoints([point6, point7, point8, point9, point10]);
        pList = curve1.concat(curve2);
        pList.push(point11);
      }

      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return CurveFlag;
}();

var DoubleArrow = /*#__PURE__*/function () {
  function DoubleArrow(arg) {
    _classCallCheck(this, DoubleArrow);

    if (!arg) arg = {}; //影响因素

    var opt = {};
    opt.headHeightFactor = arg.headHeightFactor || 0.25;
    opt.headWidthFactor = arg.headWidthFactor || 0.3;
    opt.neckHeightFactor = arg.neckHeightFactor || 0.85;
    opt.neckWidthFactor = arg.neckWidthFactor || 0.15;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(DoubleArrow, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (!positions) return;
      this.positions = positions;
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1], pnts[2]];
      var pnt1 = _ref[0];
      var pnt2 = _ref[1];
      var pnt3 = _ref[2];
      var count = this.positions.length;
      var tempPoint4;
      var connPoint;

      if (count === 3) {
        tempPoint4 = this.getTempPoint4(pnt1, pnt2, pnt3);
        connPoint = this.plotUtil.Mid(pnt1, pnt2);
      } else if (count === 4) {
        tempPoint4 = pnts[3];
        connPoint = this.plotUtil.Mid(pnt1, pnt2);
      } else {
        tempPoint4 = pnts[3];
        connPoint = pnts[4];
      }

      var leftArrowPnts = undefined,
          rightArrowPnts = undefined;

      if (this.plotUtil.isClockWise(pnt1, pnt2, pnt3)) {
        leftArrowPnts = this.getArrowPoints(pnt1, connPoint, tempPoint4, false);
        rightArrowPnts = this.getArrowPoints(connPoint, pnt2, pnt3, true);
      } else {
        leftArrowPnts = this.getArrowPoints(pnt2, connPoint, pnt3, false);
        rightArrowPnts = this.getArrowPoints(connPoint, pnt1, tempPoint4, true);
      }

      var m = leftArrowPnts.length;
      var t = (m - 5) / 2;
      var llBodyPnts = leftArrowPnts.slice(0, t);
      var lArrowPnts = leftArrowPnts.slice(t, t + 5);
      var lrBodyPnts = leftArrowPnts.slice(t + 5, m);
      var rlBodyPnts = rightArrowPnts.slice(0, t);
      var rArrowPnts = rightArrowPnts.slice(t, t + 5);
      var rrBodyPnts = rightArrowPnts.slice(t + 5, m);
      rlBodyPnts = this.plotUtil.getBezierPoints(rlBodyPnts);
      var bodyPnts = this.plotUtil.getBezierPoints(rrBodyPnts.concat(llBodyPnts.slice(1)));
      lrBodyPnts = this.plotUtil.getBezierPoints(lrBodyPnts);
      var newPnts = rlBodyPnts.concat(rArrowPnts, bodyPnts, lArrowPnts, lrBodyPnts);
      var returnArr = [];

      for (var k = 0; k < newPnts.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(newPnts[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }, {
    key: "getTempPoint4",
    value: function getTempPoint4(linePnt1, linePnt2, point) {
      var midPnt = this.plotUtil.Mid(linePnt1, linePnt2);
      var len = this.plotUtil.MathDistance(midPnt, point);
      var angle = this.plotUtil.getAngleOfThreePoints(linePnt1, midPnt, point);
      var symPnt = undefined,
          distance1 = undefined,
          distance2 = undefined,
          mid = undefined;

      if (angle < Math.PI / 2) {
        distance1 = len * Math.sin(angle);
        distance2 = len * Math.cos(angle);
        mid = this.plotUtil.getThirdPoint(linePnt1, midPnt, Math.PI / 2, distance1, false);
        symPnt = this.plotUtil.getThirdPoint(midPnt, mid, Math.PI / 2, distance2, true);
      } else if (angle >= Math.PI / 2 && angle < Math.PI) {
        distance1 = len * Math.sin(Math.PI - angle);
        distance2 = len * Math.cos(Math.PI - angle);
        mid = this.plotUtil.getThirdPoint(linePnt1, midPnt, Math.PI / 2, distance1, false);
        symPnt = this.plotUtil.getThirdPoint(midPnt, mid, Math.PI / 2, distance2, false);
      } else if (angle >= Math.PI && angle < Math.PI * 1.5) {
        distance1 = len * Math.sin(angle - Math.PI);
        distance2 = len * Math.cos(angle - Math.PI);
        mid = this.plotUtil.getThirdPoint(linePnt1, midPnt, Math.PI / 2, distance1, true);
        symPnt = this.plotUtil.getThirdPoint(midPnt, mid, Math.PI / 2, distance2, true);
      } else {
        distance1 = len * Math.sin(Math.PI * 2 - angle);
        distance2 = len * Math.cos(Math.PI * 2 - angle);
        mid = this.plotUtil.getThirdPoint(linePnt1, midPnt, Math.PI / 2, distance1, true);
        symPnt = this.plotUtil.getThirdPoint(midPnt, mid, Math.PI / 2, distance2, false);
      }

      return symPnt;
    }
  }, {
    key: "getArrowPoints",
    value: function getArrowPoints(pnt1, pnt2, pnt3, clockWise) {
      var midPnt = this.plotUtil.Mid(pnt1, pnt2);
      var len = this.plotUtil.MathDistance(midPnt, pnt3);
      var midPnt1 = this.plotUtil.getThirdPoint(pnt3, midPnt, 0, len * 0.3, true);
      var midPnt2 = this.plotUtil.getThirdPoint(pnt3, midPnt, 0, len * 0.5, true);
      midPnt1 = this.plotUtil.getThirdPoint(midPnt, midPnt1, Math.PI / 2, len / 5, clockWise);
      midPnt2 = this.plotUtil.getThirdPoint(midPnt, midPnt2, Math.PI / 2, len / 4, clockWise);
      var points = [midPnt, midPnt1, midPnt2, pnt3];
      var arrowPnts = this.plotUtil.getArrowHeadPointsNoLR(points);

      if (arrowPnts && Array.isArray(arrowPnts) && arrowPnts.length > 0) {
        var _ref2 = [arrowPnts[0], arrowPnts[4]],
            neckLeftPoint = _ref2[0],
            neckRightPoint = _ref2[1];
        var tailWidthFactor = this.plotUtil.MathDistance(pnt1, pnt2) / this.plotUtil.getBaseLength(points) / 2;
        var bodyPnts = this.plotUtil.getArrowBodyPoints(points, neckLeftPoint, neckRightPoint, tailWidthFactor);

        if (bodyPnts) {
          var n = bodyPnts.length;
          var lPoints = bodyPnts.slice(0, n / 2);
          var rPoints = bodyPnts.slice(n / 2, n);
          lPoints.push(neckLeftPoint);
          rPoints.push(neckRightPoint);
          lPoints = lPoints.reverse();
          lPoints.push(pnt2);
          rPoints = rPoints.reverse();
          rPoints.push(pnt1);
          return lPoints.reverse().concat(arrowPnts, rPoints);
        }
      } else {
        throw new Error('插值出错');
      }
    }
  }]);

  return DoubleArrow;
}();

var FineArrow = /*#__PURE__*/function () {
  function FineArrow(arg) {
    _classCallCheck(this, FineArrow);

    if (!arg) arg = {}; //影响因素

    var opt = {};
    opt.headAngle = this.headAngle = arg.headAngle || Math.PI / 8.5;
    opt.neckAngle = this.neckAngle = arg.neckAngle || Math.PI / 13;
    opt.tailWidthFactor = this.tailWidthFactor = arg.tailWidthFactor || 0.1;
    opt.neckWidthFactor = this.neckWidthFactor = arg.neckWidthFactor || 0.2;
    opt.headWidthFactor = this.headWidthFactor = arg.headWidthFactor || 0.25;
    opt.neckHeightFactor = arg.neckHeightFactor || 0.85;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(FineArrow, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (!positions) return;
      this.positions = positions;
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1]],
          pnt1 = _ref[0],
          pnt2 = _ref[1];
      var len = this.plotUtil.getBaseLength(pnts);
      var tailWidth = len * this.tailWidthFactor;
      var neckWidth = len * this.neckWidthFactor;
      var headWidth = len * this.headWidthFactor;
      var tailLeft = this.plotUtil.getThirdPoint(pnt2, pnt1, Math.PI / 2, tailWidth, true);
      var tailRight = this.plotUtil.getThirdPoint(pnt2, pnt1, Math.PI / 2, tailWidth, false);
      var headLeft = this.plotUtil.getThirdPoint(pnt1, pnt2, this.headAngle, headWidth, false);
      var headRight = this.plotUtil.getThirdPoint(pnt1, pnt2, this.headAngle, headWidth, true);
      var neckLeft = this.plotUtil.getThirdPoint(pnt1, pnt2, this.neckAngle, neckWidth, false);
      var neckRight = this.plotUtil.getThirdPoint(pnt1, pnt2, this.neckAngle, neckWidth, true);
      var pList = [tailLeft, neckLeft, headLeft, pnt2, headRight, neckRight, tailRight];
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return FineArrow;
}();

var FineArrowYW = /*#__PURE__*/function () {
  function FineArrowYW(arg) {
    _classCallCheck(this, FineArrowYW);

    if (!arg) arg = {}; //影响因素

    var opt = {};
    opt.headHeightFactor = arg.headHeightFactor || 0.18;
    opt.headWidthFactor = arg.headWidthFactor || 0.3;
    opt.neckHeightFactor = arg.neckHeightFactor || 0.85;
    opt.neckWidthFactor = arg.neckWidthFactor || 0.15;
    opt.tailWidthFactor = this.tailWidthFactor = arg.tailWidthFactor || 0.1;
    opt.swallowTailFactor = this.swallowTailFactor = arg.swallowTailFactor || 1;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(FineArrowYW, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (!positions) return;
      this.positions = positions;
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var tailPnts = this.getTailPoints(pnts);
      var headPnts = this.plotUtil.getArrowHeadPoints(pnts, tailPnts[0], tailPnts[2]);
      var neckLeft = headPnts[0];
      var neckRight = headPnts[4];
      var bodyPnts = this.plotUtil.getArrowBodyPoints(pnts, neckLeft, neckRight, this.tailWidthFactor);
      var _count = bodyPnts.length;
      var leftPnts = [tailPnts[0]].concat(bodyPnts.slice(0, _count / 2));
      leftPnts.push(neckLeft);
      var rightPnts = [tailPnts[2]].concat(bodyPnts.slice(_count / 2, _count));
      rightPnts.push(neckRight);
      leftPnts = this.plotUtil.getQBSplinePoints(leftPnts);
      rightPnts = this.plotUtil.getQBSplinePoints(rightPnts);
      var pList = leftPnts.concat(headPnts, rightPnts.reverse(), [tailPnts[1], leftPnts[0]]);
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }, {
    key: "getTailPoints",
    value: function getTailPoints(points) {
      var allLen = this.plotUtil.getBaseLength(points);
      var tailWidth = allLen * this.tailWidthFactor;
      var tailLeft = this.plotUtil.getThirdPoint(points[1], points[0], Math.PI / 2, tailWidth, false);
      var tailRight = this.plotUtil.getThirdPoint(points[1], points[0], Math.PI / 2, tailWidth, true);
      var len = tailWidth * this.swallowTailFactor;
      var swallowTailPnt = this.plotUtil.getThirdPoint(points[1], points[0], 0, len, true);
      return [tailLeft, swallowTailPnt, tailRight];
    }
  }]);

  return FineArrowYW;
}();
/* 集结地 */


var GatheringPlace = /*#__PURE__*/function () {
  function GatheringPlace(opt) {
    _classCallCheck(this, GatheringPlace);

    if (!opt) opt = {}; //影响因素

    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(GatheringPlace, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var mid = this.plotUtil.Mid(pnts[0], pnts[2]);
      pnts.push(mid, pnts[0], pnts[1]);
      var normals = [],
          pnt1 = undefined,
          pnt2 = undefined,
          pnt3 = undefined,
          pList = [];

      for (var i = 0; i < pnts.length - 2; i++) {
        pnt1 = pnts[i];
        pnt2 = pnts[i + 1];
        pnt3 = pnts[i + 2];
        var normalPoints = this.plotUtil.getBisectorNormals(0.4, pnt1, pnt2, pnt3);
        normals = normals.concat(normalPoints);
      }

      var count = normals.length;
      normals = [normals[count - 1]].concat(normals.slice(0, count - 1));

      for (var _i = 0; _i < pnts.length - 2; _i++) {
        pnt1 = pnts[_i];
        pnt2 = pnts[_i + 1];
        pList.push(pnt1);

        for (var t = 0; t <= 100; t++) {
          var _pnt = this.plotUtil.getCubicValue(t / 100, pnt1, normals[_i * 2], normals[_i * 2 + 1], pnt2);

          pList.push(_pnt);
        }

        pList.push(pnt2);
      }

      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return GatheringPlace;
}();
/* 直线箭头 */


var LineStraightArrow = /*#__PURE__*/function () {
  function LineStraightArrow(arg) {
    _classCallCheck(this, LineStraightArrow);

    var opt = {}; //影响因素

    this.typeName = "LineStraightArrow";
    this.plotUtil = new ArrowUtil(opt);
    this.fixPointCount = 2;
    this.maxArrowLength = 3000000;
    this.arrowLengthScale = 5;
  }

  _createClass(LineStraightArrow, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var pList = [];

      try {
        if (pnts.length < 2) {
          return false;
        } else {
          var _ref = [pnts[0], pnts[1]],
              pnt1 = _ref[0],
              pnt2 = _ref[1];
          var distance = this.plotUtil.MathDistance(pnt1, pnt2);
          var len = distance / this.arrowLengthScale;
          len = len > this.maxArrowLength ? this.maxArrowLength : len;
          var leftPnt = this.plotUtil.getThirdPoint(pnt1, pnt2, Math.PI / 6, len, false);
          var rightPnt = this.plotUtil.getThirdPoint(pnt1, pnt2, Math.PI / 6, len, true);
          pList = [pnt1, pnt2, leftPnt, pnt2, rightPnt];
        }
      } catch (e) {
        console.log(e);
      }

      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return LineStraightArrow;
}();
/* 弓形面 */


var Lune = /*#__PURE__*/function () {
  function Lune(opt) {
    _classCallCheck(this, Lune);

    if (!opt) opt = {}; //影响因素

    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(Lune, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1], pnts[2], undefined, undefined],
          pnt1 = _ref[0],
          pnt2 = _ref[1],
          pnt3 = _ref[2],
          startAngle = _ref[3],
          endAngle = _ref[4];
      var center = this.plotUtil.getCircleCenterOfThreePoints(pnt1, pnt2, pnt3);
      var radius = this.plotUtil.MathDistance(pnt1, center);
      var angle1 = this.plotUtil.getAzimuth(pnt1, center);
      var angle2 = this.plotUtil.getAzimuth(pnt2, center);

      if (this.plotUtil.isClockWise(pnt1, pnt2, pnt3)) {
        startAngle = angle2;
        endAngle = angle1;
      } else {
        startAngle = angle1;
        endAngle = angle2;
      }

      pnts = this.plotUtil.getArcPoints(center, radius, startAngle, endAngle);
      pnts.push(pnts[0]);
      var returnArr = [];

      for (var k = 0; k < pnts.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pnts[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return Lune;
}();
/* 三角旗 */


var RectFlag = /*#__PURE__*/function () {
  function RectFlag(opt) {
    _classCallCheck(this, RectFlag);

    if (!opt) opt = {}; //影响因素

    opt.typeName = "RectFlag";
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(RectFlag, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var components = [];

      if (pnts.length > 1) {
        var startPoint = pnts[0];
        var endPoint = pnts[pnts.length - 1];
        var point1 = [endPoint[0], startPoint[1]];
        var point2 = [endPoint[0], (startPoint[1] + endPoint[1]) / 2];
        var point3 = [startPoint[0], (startPoint[1] + endPoint[1]) / 2];
        var point4 = [startPoint[0], endPoint[1]];
        components = [startPoint, point1, point2, point3, point4];
      }

      var returnArr = [];

      for (var k = 0; k < components.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(components[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return RectFlag;
}();
/* 扇形 */


var Sector = /*#__PURE__*/function () {
  function Sector(arg) {
    _classCallCheck(this, Sector);

    var opt = {}; //影响因素

    this.typeName = "Sector";
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(Sector, [{
    key: "startCompute",
    value: function startCompute(positions) {
      if (positions.length <= 2) return [];
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1], pnts[2]],
          center = _ref[0],
          pnt2 = _ref[1],
          pnt3 = _ref[2];
      var radius = this.plotUtil.MathDistance(pnt2, center);
      var startAngle = this.plotUtil.getAzimuth(pnt2, center);
      var endAngle = this.plotUtil.getAzimuth(pnt3, center);
      var pList = this.plotUtil.getArcPoints(center, radius, startAngle, endAngle);
      pList.push(center, pList[0]);
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return Sector;
}();

var StraightArrow = /*#__PURE__*/function () {
  function StraightArrow(arg) {
    _classCallCheck(this, StraightArrow);

    if (!arg) arg = {}; //影响因素

    var opt = {};
    opt.tailWidthFactor = this.tailWidthFactor = arg.tailWidthFactor || 0.05;
    opt.neckWidthFactor = this.neckWidthFactor = arg.neckWidthFactor || 0.1;
    opt.headWidthFactor = this.headWidthFactor = arg.headWidthFactor || 0.15;
    this.headAngle = Math.PI / 4;
    this.neckAngle = Math.PI * 0.17741;
    this.positions = null;
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(StraightArrow, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var _ref = [pnts[0], pnts[1]],
          pnt1 = _ref[0],
          pnt2 = _ref[1];
      var len = this.plotUtil.getBaseLength(pnts);
      var tailWidth = len * this.tailWidthFactor;
      var neckWidth = len * this.neckWidthFactor;
      var headWidth = len * this.headWidthFactor;
      var tailLeft = this.plotUtil.getThirdPoint(pnt2, pnt1, Math.PI / 2, tailWidth, true);
      var tailRight = this.plotUtil.getThirdPoint(pnt2, pnt1, Math.PI / 2, tailWidth, false);
      var headLeft = this.plotUtil.getThirdPoint(pnt1, pnt2, this.headAngle, headWidth, false);
      var headRight = this.plotUtil.getThirdPoint(pnt1, pnt2, this.headAngle, headWidth, true);
      var neckLeft = this.plotUtil.getThirdPoint(pnt1, pnt2, this.neckAngle, neckWidth, false);
      var neckRight = this.plotUtil.getThirdPoint(pnt1, pnt2, this.neckAngle, neckWidth, true);
      var pList = [tailLeft, neckLeft, headLeft, pnt2, headRight, neckRight, tailRight];
      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return StraightArrow;
}();

var TrangleFlag = /*#__PURE__*/function () {
  function TrangleFlag(arg) {
    _classCallCheck(this, TrangleFlag);

    var opt = {}; //影响因素

    this.typeName = "TrangleFlag";
    this.plotUtil = new ArrowUtil(opt);
  }

  _createClass(TrangleFlag, [{
    key: "startCompute",
    value: function startCompute(positions) {
      var pnts = [];

      for (var i = 0; i < positions.length; i++) {
        var newP = this.plotUtil.cartesian32WeMercator(positions[i]);
        pnts.push(newP);
      }

      var pList = [];

      if (pnts.length > 1) {
        var startPoint = pnts[0];
        var endPoint = pnts[pnts.length - 1];
        var point1 = [endPoint[0], (startPoint[1] + endPoint[1]) / 2];
        var point2 = [startPoint[0], (startPoint[1] + endPoint[1]) / 2];
        var point3 = [startPoint[0], endPoint[1]];
        pList = [startPoint, point1, point2, point3];
      }

      var returnArr = [];

      for (var k = 0; k < pList.length; k++) {
        var posi = this.plotUtil.webMercator2Cartesian3(pList[k]);
        returnArr.push(posi);
      }

      return returnArr;
    }
  }]);

  return TrangleFlag;
}();

var ArrowAlgorithm = {
  AttackArrow: AttackArrow,
  AttackArrowPW: AttackArrowPW,
  AttackArrowYW: AttackArrowYW,
  CloseCurve: CloseCurve,
  Curve: Curve,
  CurveFlag: CurveFlag,
  DoubleArrow: DoubleArrow,
  FineArrow: FineArrow,
  FineArrowYW: FineArrowYW,
  GatheringPlace: GatheringPlace,
  StraightArrow: StraightArrow,
  LineStraightArrow: LineStraightArrow,
  TrangleFlag: TrangleFlag,
  Lune: Lune,
  RectFlag: RectFlag,
  Sector: Sector
};

/* 构建军事标绘 */

/**
 * 军事标绘类
 * @class
 * @augments BasePlot
 * @alias BasePlot.CreateArrow
 */

var CreateArrow = /*#__PURE__*/function (_BasePlot) {
  _inherits(CreateArrow, _BasePlot);

  var _super = _createSuper(CreateArrow);

  function CreateArrow(viewer, opt) {
    var _this;

    _classCallCheck(this, CreateArrow);

    _this = _super.call(this, viewer, opt);
    _this.opt = opt || {};

    var _ref = _this.opt || {},
        arrowType = _ref.arrowType,
        style = _ref.style;
    /**
     * @property {String} type 标绘类型
     */


    _this.type = "arrow";

    if (!arrowType) {
      console.log("缺少箭头类型");
      return _possibleConstructorReturn(_this);
    }
    /**
     * @property {String} arrowType 箭头类型（1~攻击箭头/2~攻击箭头平尾/3~攻击箭头燕尾/4~闭合曲面/5~钳击箭头/6~单尖直箭头/7~粗单尖直箭头/8~集结地/9~弓形面/10~粗直箭头/11~矩形棋/12~扇形/13~三角旗/14~曲线旗/15~曲线/16~单线箭头）
     */


    _this.arrowType = arrowType;
    _this.arrowObj = getSituationByType(arrowType);
    if (!_this.arrowObj) return _possibleConstructorReturn(_this);
    _this.minPointNum = _this.arrowObj.minPointNum;

    if (_this.minPointNum == 1) {
      console.warn("控制点有误！");
      return _possibleConstructorReturn(_this);
    }

    _this.maxPointNum = _this.arrowObj.maxPointNum == -1 ? _this.minPointNum : _this.arrowObj.maxPointNum;
    /**
     * @property {Object} arrowPlot 箭头标绘对象
     */

    _this.arrowPlot = _this.arrowObj.arrowPlot;

    if (!_this.arrowPlot) {
      console.warn("计算坐标类有误！");
      return _possibleConstructorReturn(_this);
    }

    _this.viewer = viewer;
    /**
     * @property {Cesium.Entity} entity 箭头实体
     */

    _this.entity = null;
    _this.polyline = null;
    var defaultStyle = {
      outlineColor: "#ff0000",
      outlineWidth: 2
    };
    /**
     * @property {Object} style 样式
    */

    _this.style = Object.assign(defaultStyle, style || {});
    _this.outline = null;
    return _this;
  }
  /**
   * 开始绘制
   * @param {Function} callback 绘制成功后回调函数
  */


  _createClass(CreateArrow, [{
    key: "start",
    value: function start(callback) {
      var that = this;
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        //单机开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;
        if (that.positions.length > that.maxPointNum) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        that.positions.push(cartesian);
        var point = that.createPoint(cartesian);
        point.wz = that.controlPoints.length;
        that.controlPoints.push(point);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //移动时绘制面
        if (that.positions.length < 1) {
          if (that.prompt) that.prompt.update(evt.endPosition, "单击开始绘制");
          that.state = "startCreate";
          return;
        }

        if (that.positions.length == that.maxPointNum) {
          if (that.prompt) that.prompt.update(evt.endPosition, "双击结束");
        } else if (that.positions.length > that.maxPointNum) {
          //sin add 20240710
          if(that.arrowType == 7){
            that.movePush = false;
            var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
            if(cartesian) {
              that.controlPoints.push(cartesian);
            }
          }
          that.endCreate();
          if (callback && that.entity) callback(that.entity);
          return;
        } else {
          if (that.prompt) that.prompt.update(evt.endPosition, "单击新增，不少于" + that.minPointNum + "个点</br>" + "双击结束");
        }

        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (!that.movePush) {
          that.positions.push(cartesian);
          that.movePush = true;
        } else {
          that.positions[that.positions.length - 1] = cartesian;
        }

        if (that.positions.length >= 2 && !Cesium.defined(that.polyline)) that.polyline = that.createPolyline(1);
        //sin update 20240710
        if (that.positions.length >= that.minPointNum && that.positions.length <= that.maxPointNum && !Cesium.defined(that.entity)) {
          that.entity = that.createEntity(1);
          that.entity.objId = that.objId;
          that.polyline.show = false;
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
 /*       if (!that.entity) return;
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer, [that.entity]);
        if (!cartesian) return;
        if (that.positions.length >= that.minPointNum) that.endCreate(callback);
        if (callback) callback(that.entity);*/
        //sin add 20240710
        if (that.positions.length > that.minPointNum){
          that.movePush = false;
          that.endCreate();
          if (callback && that.entity) callback(that.entity);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      if (!that.movePush) {
        // 双击结束
        that.positions.pop();
        that.movePush = false;
        that.viewer.entities.remove(that.controlPoints[that.controlPoints.length - 1]);
        that.controlPoints.pop();
      }
      //sin add 20240710
      if(this.polyline){
        that.viewer.entities.remove(this.polyline);
      }
      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.handler.destroy();
      that.state = "endCreate";
    }
    /**
     * 当前步骤结束
     */

  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
    /**
     * 通过坐标数组构建
     * @param {Array} lnglatArr 经纬度坐标数组
     * @callback {Function} callback 绘制成功后回调函数
    */

  }, {
    key: "createByPositions",
    value: function createByPositions(lnglatArr, callback) {
      //通过传入坐标数组创建面
      if (!lnglatArr) return;
      this.state = "startCreate";
      var positions = lnglatArr[0] instanceof Cesium.Cartesian3 ? lnglatArr : util$1.lnglatsToCartesians(lnglatArr);
      if (!positions) return;
      this.positions = positions;
      this.entity = this.createEntity();

      for (var i = 0; i < positions.length; i++) {
        var newP = positions[i];
        var point = this.createPoint(newP);
        var ctgc = Cesium.Cartographic.fromCartesian(positions[i]);
        point.point.heightReference = this.style.heightReference;
        point.ctgc = ctgc;
        point.wz = this.controlPoints.length;
        this.controlPoints.push(point);
      }

      this.state = "endCreate";
      this.entity.objId = this.objId;
      if (callback) callback(this.entity);
    }
    /**
     * 获取样式
     * @returns {Object} 样式
    */

  }, {
    key: "getStyle",
    value: function getStyle() {
      if (!this.entity) return;
      var obj = {};
      var entity = undefined;

      if (this.arrowPlot.onlyLine) {
        entity = this.entity.polyline;
      } else {
        entity = this.entity.polygon;
        obj.fill = entity.fill ? entity.fill.getValue() : false;
      }

      var color = entity.material.color.getValue();
      obj.colorAlpha = color.alpha;
      obj.color = new Cesium.Color(color.red, color.green, color.blue, 1).toCssHexString();

      if (this.arrowPlot.onlyLine) {
        var heightReference = entity.clampToGround.getValue();
        obj.heightReference = Number(heightReference);
      } else {
        var _heightReference = entity.heightReference.getValue();

        obj.heightReference = Number(_heightReference);
      }

      return obj;
    }
    /**
     * 设置相关样式
     * @param {Object} style 样式
     */

  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      var color = style.color instanceof Cesium.Color ? style.color : Cesium.Color.fromCssColorString(style.color);
      var material = color.withAlpha(style.colorAlpha || 1);

      if (this.arrowPlot.onlyLine) {
        this.entity.polyline.material = material;
        this.entity.polyline.clampToGround = Boolean(style.heightReference);
      } else if (this.arrowPlot.hasLine) {
        this.entity.polyline.material = material;
        this.entity.polygon.material = material;
        this.entity.polyline.clampToGround = Boolean(style.heightReference);
        this.entity.polygon.heightReference = Number(style.heightReference);
      } else {
        if (style.fill != undefined) this.entity.polygon.fill = style.fill;
        this.entity.polygon.material = material;
        this.entity.polygon.heightReference = Number(style.heightReference);
      }

      this.style = Object.assign(this.style, style);
    } // 构建态势标绘面

  }, {
    key: "changeProperty",
    value: function changeProperty(code) {
      var that = this
      if(!that.entity) return
      if(code == 1) {
        if(that.polyline) {
          let l1 = new Cesium.CallbackProperty(() => {
            return that.positions;
          }, false);
          that.polyline._polyline.positions = l1;
        }

        if (that.arrowPlot.hasLine) {
          
          let pH
          pH = new Cesium.CallbackProperty(function () {
            let newPosition = that.arrowPlot.startCompute(that.positions);
            if (that.arrowPlot.spliceWZ !== null) {
              newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
            }
            return new Cesium.PolygonHierarchy(newPosition);
          }, false)

          
          let lP
          lP = new Cesium.CallbackProperty(function () {
            let newPositionL = that.arrowPlot.startCompute(that.positions);
            if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
              var arr = [];

              for (var i = 0; i < that.arrowPlot.lineWZ.length; i++) {
                arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
              }

              return arr;
            } else {
              return newPositionL;
            }
          }, false)
          that.entity._polygon.hierarchy = pH;
          that.entity._polyline.positions = lP;
        } else if (that.arrowPlot.onlyLine) {
          let lP
          lP = new Cesium.CallbackProperty(function () {
            let newPositionL = that.arrowPlot.startCompute(that.positions);
            if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
              var arr = [];

              for (var i = 0; i < that.arrowPlot.lineWZ.length; i++) {
                arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
              }

              return arr;
            } else {
              return newPositionL;
            }
          }, false)
          that.entity._polyline.positions = lP;
        }else {
          let pH
          pH = new Cesium.CallbackProperty(function () {
            let newPosition = that.arrowPlot.startCompute(that.positions);
            if (that.arrowPlot.spliceWZ != undefined) {
              newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
            }
            return new Cesium.PolygonHierarchy(newPosition);
          }, false)
          that.entity._polygon.hierarchy = pH;
        }

      }else {
        if (that.arrowPlot.hasLine) {
          let newPosition = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.spliceWZ !== null) {
            newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
          }
          let pH
          pH = new Cesium.PolygonHierarchy(newPosition);
          
          let newPositionL = that.arrowPlot.startCompute(that.positions);
          let lP
          if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
            let arr = [];

            for (let i = 0; i < that.arrowPlot.lineWZ.length; i++) {
              arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
            }

            lP = arr;
          } else {
            lP = newPositionL;
          }
          that.entity._polygon.hierarchy = pH;
          that.entity._polyline.positions = lP;
        } else if(that.arrowPlot.onlyLine) {
          let newPositionL = that.arrowPlot.startCompute(that.positions);
          let lP
          if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
            let arr = [];

            for (let i = 0; i < that.arrowPlot.lineWZ.length; i++) {
              arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
            }

            lP = arr;
          } else {
            lP = newPositionL;
          }
          that.entity._polyline.positions = lP;
        } else {
          let newPosition = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.spliceWZ != undefined) {
            newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
          }
          let pH
          pH = new Cesium.PolygonHierarchy(newPosition);
          that.entity._polygon.hierarchy = pH;
        }
        if(that.polyline) {      
          that.polyline._polyline.positions = that.positions;
        }
      }
    }
  },
  {
    key: "createEntity",
    value: function createEntity(code) {
      var that = this;
      this.style.color = this.style.color || Cesium.Color.WHITE;
      this.style.outlineColor = this.style.outlineColor || Cesium.Color.BLACK;
      var color = this.style.color instanceof Cesium.Color ? this.style.color : Cesium.Color.fromCssColorString(this.style.color).withAlpha(this.style.colorAlpha || 1);
      var entityObj = undefined;

      if (that.arrowPlot.hasLine) {
        
        let pH
        if(code) {
          pH = new Cesium.CallbackProperty(function () {
            let newPosition = that.arrowPlot.startCompute(that.positions);
            if (that.arrowPlot.spliceWZ !== null) {
              newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
            }
            return new Cesium.PolygonHierarchy(newPosition);
          }, false)
        }else {
          let newPosition = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.spliceWZ !== null) {
            newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
          }
          pH = new Cesium.PolygonHierarchy(newPosition);
        }

        let lP

        if(code) {
          lP = new Cesium.CallbackProperty(function () {
            let newPositionL = that.arrowPlot.startCompute(that.positions);

            if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
              var arr = [];

              for (var i = 0; i < that.arrowPlot.lineWZ.length; i++) {
                arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
              }

              return arr;
            } else {
              return newPositionL;
            }
          }, false)
        } else {
          let newPositionL = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
            var arr = [];

            for (var i = 0; i < that.arrowPlot.lineWZ.length; i++) {
              arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
            }

            lP = arr;
          } else {
            lP = newPositionL;
          }
        }

        // 线面混合
        entityObj = {
          polygon: {
            hierarchy: pH,
            heightReference: this.style.heightReference == undefined ? 0 : 1,
            material: color
          },
          polyline: {
            positions: lP,
            material: color,
            clampToGround: this.style.heightReference == undefined ? false : true,
            width: 3
          }
        };
      } else if (that.arrowPlot.onlyLine) {
        let lP
        if(code) {
          lP = new Cesium.CallbackProperty(function () {
            let newPositionL = that.arrowPlot.startCompute(that.positions);
            if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
              var arr = [];

              for (var i = 0; i < that.arrowPlot.lineWZ.length; i++) {
                arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
              }

              return arr;
            } else {
              return newPositionL;
            }
          }, false)
          
        }
        else {
          let newPositionL = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.lineWZ && that.arrowPlot.lineWZ.length > 0) {
            let arr = [];

            for (let i = 0; i < that.arrowPlot.lineWZ.length; i++) {
              arr.push(newPositionL[that.arrowPlot.lineWZ[i] - 1]);
            }

            lP = arr;
          } else {
            lP = newPositionL;
          }
          
        }
        // 只有线
        entityObj = {
          polyline: {
            positions: lP,
            material: color,
            clampToGround: this.style.heightReference == undefined ? false : true,
            width: 3
          }
        };
      } else {
        // 只有面
        let pH
        if(code) {
          pH = new Cesium.CallbackProperty(function () {
          let newPosition = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.spliceWZ != undefined) {
            newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
          }
            return new Cesium.PolygonHierarchy(newPosition);
          }, false)
        }else {
          let newPosition = that.arrowPlot.startCompute(that.positions);
          if (that.arrowPlot.spliceWZ != undefined) {
            newPosition.splice(that.arrowPlot.spliceWZ - 1, 1);
          }
          pH = new Cesium.PolygonHierarchy(newPosition);
        }
        
        entityObj = {
          polygon: {
            hierarchy: pH,
            heightReference: Number(this.style.heightReference),
            show: true,
            fill: this.style.fill || true,
            material: color
          }
        };
        /* if (!this.style.heightReference) {
        		entityObj.polygon.height = 0; // 不贴地 必设
        		entityObj.polygon.perPositionHeight = true; // 启用点的真实高度
        	} */
      }

      return this.viewer.entities.add(entityObj);
    }
  }, 
  {
    key: "createPolyline",
    value: function createPolyline(code) {
      var that = this;
      let newPositions
      if(code) {
        newPositions =  new Cesium.CallbackProperty(function () {
          return that.positions
        }, false)
      }else {
        newPositions = that.positions
      }
      return this.viewer.entities.add({
        polyline: {
          positions: newPositions,
          clampToGround: Boolean(this.style.clampToGround),
          material: this.style.outlineColor instanceof Cesium.Color ? this.style.outlineColor : Cesium.Color.fromCssColorString(this.style.outlineColor).withAlpha(this.style.outlineColorAlpha || 1),
          width: this.style.outlineWidth || 1
        }
      });
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      if (this.entity) {
        this.viewer.entities.remove(this.entity);
        this.entity = null;
      }

      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      this.positions = [];
      this.style = null;

      if (this.modifyPoint) {
        this.viewer.entities.remove(this.modifyPoint);
        this.modifyPoint = null;
      }

      for (var i = 0; i < this.controlPoints.length; i++) {
        var point = this.controlPoints[i];
        this.viewer.entities.remove(point);
      }

      this.controlPoints = [];
      this.state = "no";
      if (this.prompt) this.prompt.destroy();

      if (this.polyline) {
        this.polyline = null;
        this.viewer.entities.remove(this.polyline);
      }

      this.forbidDrawWorld(false);
    }
  }]);

  return CreateArrow;
}(BasePlot);

function getSituationByType(type) {
  type = Number(type);

  if (isNaN(type)) {
    console.warn("输入态势标绘类型不对！");
    return;
  }

  if (!type || typeof type != "number") {
    console.warn("输入态势标绘类型不对！");
    return;
  }

  var arrowPlot;
  var minPointNum = -1;
  var maxPointNum = -1;
  var playObj = {
    canPlay: false,
    // 是否可移动
    pointNum: 0,
    // 可移动的点的数量
    pointWZ: [] // 可移动的点在数组中的位置 从0开始

  };
  playObj.canPlay = false; // 是否可以自动播放

  switch (type) {
    case 1:
      arrowPlot = new ArrowAlgorithm.AttackArrow(); // 攻击箭头

      minPointNum = 3;
      maxPointNum = 999;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 2:
      arrowPlot = new ArrowAlgorithm.AttackArrowPW(); //攻击箭头平尾

      minPointNum = 3;
      maxPointNum = 999;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 3:
      arrowPlot = new ArrowAlgorithm.AttackArrowYW(); //攻击箭头燕尾

      minPointNum = 3;
      maxPointNum = 999;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 4:
      arrowPlot = new ArrowAlgorithm.CloseCurve(); //闭合曲面

      minPointNum = 3;
      maxPointNum = 999;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 5:
      arrowPlot = new ArrowAlgorithm.DoubleArrow(); //钳击箭头

      minPointNum = 3; // 最小可为三个点 为做动画效果 故写死为5个点

      maxPointNum = 5;
      playObj.canPlay = true;
      playObj.pointNum = 2;
      playObj.pointWZ = [2, 3];
      break;

    case 6:
      arrowPlot = new ArrowAlgorithm.FineArrow(); //单尖直箭头

      minPointNum = 2;
      maxPointNum = 2;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 7:
      arrowPlot = new ArrowAlgorithm.FineArrowYW(); //粗单尖直箭头(带燕尾)

      minPointNum = 2;
      maxPointNum = 2;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 8:
      arrowPlot = new ArrowAlgorithm.GatheringPlace(); //集结地

      minPointNum = 3;
      maxPointNum = 3;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 9:
      arrowPlot = new ArrowAlgorithm.Lune(); //弓形面

      minPointNum = 3;
      playObj.canPlay = true;
      maxPointNum = 3;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 10:
      arrowPlot = new ArrowAlgorithm.StraightArrow(); //粗直箭头

      minPointNum = 2;
      maxPointNum = 2;
      playObj.canPlay = true;
      playObj.pointNum = 1;
      playObj.pointWZ = [maxPointNum];
      break;

    case 11:
      arrowPlot = new ArrowAlgorithm.RectFlag(); //矩形旗

      minPointNum = 2;
      maxPointNum = 2;
      arrowPlot.hasLine = true;
      arrowPlot.lineWZ = [1, 4, 5]; // 线坐标位置

      arrowPlot.spliceWZ = [5]; // 面所需要去除点的坐标位置

      playObj.canPlay = false;
      break;

    case 12:
      arrowPlot = new ArrowAlgorithm.Sector(); //扇形

      minPointNum = 3;
      maxPointNum = 3;
      playObj.canPlay = false;
      break;

    case 13:
      arrowPlot = new ArrowAlgorithm.TrangleFlag(); //三角旗

      minPointNum = 2;
      maxPointNum = 2;
      arrowPlot.hasLine = true;
      arrowPlot.lineWZ = [1, 3, 4]; // 线坐标位置

      arrowPlot.spliceWZ = [4]; // 面所需要去除点的坐标位

      playObj.canPlay = false;
      break;

    case 14:
      arrowPlot = new ArrowAlgorithm.CurveFlag(); //曲线旗

      minPointNum = 2;
      maxPointNum = 2;
      arrowPlot.hasLine = true;
      arrowPlot.lineWZ = [1, 202, 203]; // 线坐标位置

      arrowPlot.spliceWZ = [203]; // 面所需要去除点的坐标位

      playObj.canPlay = false;
      break;

    case 15:
      arrowPlot = new ArrowAlgorithm.Curve(); //曲线

      minPointNum = 2;
      maxPointNum = 999;
      arrowPlot.onlyLine = true;
      playObj.canPlay = true;
      break;

    case 16:
      arrowPlot = new ArrowAlgorithm.LineStraightArrow(); //单线箭头

      minPointNum = 2;
      maxPointNum = 2;
      arrowPlot.onlyLine = true;
      playObj.canPlay = true;
      break;

    default:
      console.warn("不存在该类型！");
      break;
  }

  return {
    arrowPlot: arrowPlot,
    minPointNum: minPointNum,
    maxPointNum: maxPointNum,
    playObj: playObj
  };
}

/**
 * 绘制控制类
 * 
 * @class
 * @example
 * let drawTool = new vis3d.DrawTool(window.viewer, {
    canEdit: true,
  });
  plotDrawTool.on("endCreate", function (entObj, ent) {});
  plotDrawTool.start({
      "name": "面",
      "type": "polygon",
      "style": {
          "color": "#0000ff",
          "outline": true,
          "outlineColor": "#ff0000",
          "heightReference": 1
      }
  })
 */

var DrawTool = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.viewer} viewer 地图viewer对象 
   * @param {Object} obj 相关属性配置
   * @param {Boolean} obj.canEdit 是否可编辑
   */
  function DrawTool(viewer, obj) {
    _classCallCheck(this, DrawTool);

    if (!viewer) {
      console.warn("缺少必要参数！--viewer");
      return;
    }

    obj = obj || {};
    this.viewer = viewer;
    /**
     * 
     * @property {String} plotId 标绘工具id
     */

    this.plotId = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    /**
     * 
     * @property {Array} entityObjArr 标绘对象数组
     */

    this.entityObjArr = [];
    this.handler = null;
    this.removeHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    /* this.show = obj.drawEndShow == undefined ? true : obj.drawEndShow; */

    /**
     * @property {Object} nowEditEntityObj 当前编辑对象
     */

    this.startEditFun = null;
    this.endEditFun = null;
    this.removeFun = null;
    this.editingFun = undefined;
    this.deleteEntityObj = null; // 无论如何 进来先监听点击修改 与 右键删除事件 通过控制canEdit来判断要不要向下执行

    this.bindEdit();
    this.bindRemove();
    /**
     * @property {Boolean} canEdit 绘制的对象，是否可编辑
     */

    this.canEdit = obj.canEdit == undefined ? true : obj.canEdit;

    /**
     * @property {Boolean} canRemove 绘制的对象，是否可编辑
     */

    this.canRemove = obj.canRemove == undefined ? true : obj.canRemove;

    /**
     * @property {Boolean} fireEdit 绘制的对象，是否直接进入编辑状态（需要canEdit==true）
     */

    this.fireEdit = obj.fireEdit == undefined ? true : obj.fireEdit;
    this.nowDrawEntityObj = null; // 当前绘制的对象

    this.nowEditEntityObj = null; // 当前编辑的对象
  }
  /** 
   * 事件绑定
   * @param {String} type 事件类型（startEdit 开始编辑时 / endEdit 编辑结束时 / remove 删除对象时 / endCreate 创建完成后）
   * @param {Function} fun 绑定函数
  */


  _createClass(DrawTool, [{
    key: "on",
    value: function on(type, fun) {
      if (type == "startEdit") {
        // 开始编辑事件
        this.startEditFun = fun;
      }

      if (type == "endEdit") {
        // 结束编辑事件
        this.endEditFun = fun;
      }

      if (type == "remove") {
        // 移除事件
        this.removeFun = fun;
      }

      if (type == "endCreate") {
        // 绘制完成事件
        this.endCreateFun = fun;
      }

      if (type == "editing") {
        // 正在编辑
        this.editingFun = fun;
      }
    }
    /**
     * 开启编辑功能
     */

  }, {
    key: "openEdit",
    value: function openEdit() {
      this.canEdit = true;
    }
    /**
    * 关闭编辑功能
    */

  }, {
    key: "closeEdit",
    value: function closeEdit() {
      this.endEdit();
      this.canEdit = false;
    }
    /**
     * 开始绘制
     * @param {Object} opt 相关属性
     * @param {String} opt.type 绘制类型 polyline、polygon、billboard、circle、rectangle、gltfModel、point、label、arrow
     * @param {Object} opt.style 当前绘制对象的样式配置，具体配置见{@link style};
     * @returns {Object} entityObj 当前绘制对象
     */

  }, {
    key: "start",
    value: function start(opt) {
      if (!opt || !opt.type) {
        return;
      }

      opt.id = opt.id || Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0)); // 单个标绘对象id

      opt.plotId = this.plotId; // 绑定统一的工具id

      var that = this;
      this.endEdit(); // 绘制前  结束编辑

      if (this.nowDrawEntityObj && (this.nowDrawEntityObj.state == "startCreate" || this.nowDrawEntityObj.state == "creating")) {
        // 禁止一次绘制多个
        this.nowDrawEntityObj.destroy();
        this.nowDrawEntityObj = null;
      }

      var entityObj = this.createByType(opt);
      if (!entityObj) return;
      entityObj.attr = opt || {}; // 保存开始绘制时的属性
      // 开始绘制

      entityObj.start(function (entity) {
        // 绘制完成后
        that.nowDrawEntityObj = undefined;
        that.entityObjArr.push(entityObj); // endCreateFun 和 success 无本质区别，若构建时 两个都设置了 当心重复

        if (opt.success) opt.success(entityObj, entity);
        if (that.endCreateFun) that.endCreateFun(entityObj, entity);
        if (opt.show == false) entityObj.setVisible(false); // 如果可以编辑 则绘制完成打开编辑
        that.fireEdit = true; // 回显geoJson以后，编辑状态消失
        if (that.canEdit && that.fireEdit) {
          entityObj.startEdit(function () {
            if (that.editingFun) that.editingFun(entityObj, entityObj.entity);
          });
          that.nowEditEntityObj = entityObj;
          if (that.startEditFun) that.startEditFun(entityObj, entity);
        }
      });
      this.nowDrawEntityObj = entityObj;
      return entityObj;
    }
    /**
     * 结束当前操作
    */

  }, {
    key: "end",
    value: function end() {
      if (this.nowDrawEntityObj) ;
    }
    /**
    * 开始编辑绘制对象
    * @param {Object} entityObj 绘制的对象
    */

  }, {
    key: "startEditOne",
    value: function startEditOne(entityObj) {
      if (!this.canEdit) return;

      if (this.nowEditEntityObj) {
        // 结束除当前选中实体的所有编辑操作
        this.nowEditEntityObj.endEdit();

        if (this.endEditFun) {
          this.endEditFun(this.nowEditEntityObj, this.nowEditEntityObj.getEntity()); // 结束事件
        }

        this.nowEditEntityObj = null;
      }

      var that = this;

      if (entityObj) {
        entityObj.startEdit(function () {
          if (that.editingFun) that.editingFun(entityObj, entityObj.entity);
        });
        if (this.startEditFun) this.startEditFun(entityObj, entityObj.getEntity());
        this.nowEditEntityObj = entityObj;
      }
    }
    /**
     * 修改绘制对象的样式
     * @param {Object} entityObj 绘制的对象
     * @param {Object} style 样式
    */

  }, {
    key: "updateOneStyle",
    value: function updateOneStyle(entityObj, style) {
      if (entityObj) {
        entityObj.setStyle(style);
      }
    }
    /**
     * 根据坐标构建绘制对象
     * @param {Object} opt 绘制的对象
     * @param {Cesium.Cartesian3[] | Array} opt.positions 坐标数组
     * @param {Object} opt.style 当前绘制对象的样式配置，具体配置见{@link style};
     * @param {Funtion} opt.success 创建完成的回调函数
     * @param {Boolean} [opt.show] 创建完成后，是否展示
    */

  }, {
    key: "createByPositions",
    value: function createByPositions(opt) {
      opt = opt || {};
      if (!opt) opt = {};
      if (!opt.positions) return;
      opt.id = opt.id || Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
      var that = this;
      var entityObj = this.createByType(opt);
      if (!entityObj) return;
      entityObj.attr = opt; // 保存开始绘制时的属性

      entityObj.createByPositions(opt.positions, function (entity) {
        that.entityObjArr.push(entityObj);
        entityObj.setStyle(opt.style); // 设置相关样式
        // endCreateFun 和 success 无本质区别，若构建时 两个都设置了 当心重复

        if (opt.success) opt.success(entityObj, entity);
        if (that.endCreateFun) that.endCreateFun(entityObj, entity);
        if (opt.show == false) entityObj.setVisible(false); // 如果可以编辑 则绘制完成打开编辑 

        if (that.canEdit && that.fireEdit) {
          entityObj.startEdit(function () {
            if (that.editingFun) that.editingFun(entityObj, entityObj.entity);
          });
          if (that.startEditFun) that.startEditFun(entityObj, entity);
          that.nowEditEntityObj = entityObj;
        }
      });
      return entityObj;
    }
    /**
     * 由geojson格式数据创建对象
     * @param {Object} data geojson格式数据
    */

  }, {
    key: "createByGeojson",
    value: function createByGeojson(data) {
      var features = data.features;
      var entObjArr = [];

      for (var i = 0; i < features.length; i++) {
        var feature = features[i];
        var properties = feature.properties,
            geometry = feature.geometry;
        var plotType = properties.plotType;
        var geoType = geometry.type;
        var coordinates = geometry.coordinates;
        var arrowType = properties?.arrowType
        var positions = [];
        var drawType = "";

        switch (geoType) {
          case "MultiLineString":
            coordinates.forEach(function (cr) {
              positions = util$1.lnglatsToCartesians(cr);
              drawType = "polyline";
            });
            break;

          case "LineString":
            positions = util$1.lnglatsToCartesians(coordinates);
            drawType = "polyline";
            break;

          case "Polygon":
            positions = util$1.lnglatsToCartesians(coordinates[0]);
            drawType = "polygon";
            break;

          case "Point":
            positions = util$1.lnglatsToCartesians(coordinates)[0];
            drawType = plotType;
            break;
          case "arrow":
          case "rectangle":
          case "circle":
            positions = util$1.lnglatsToCartesians(coordinates);
            drawType = plotType;
            break;
        }

        this.fireEdit = false;
        var entObj = this.createByPositions({
          type: drawType,
          styleType: plotType,
          positions: positions,
          style: properties.style,
          arrowType: arrowType
        });
        entObj.attr = properties; // 保存相关信息

        if (entObj) entObjArr.push(entObj);
      }

      return entObjArr;
    }
    /**
     * 转为geojson格式
     * @returns {Object} featureCollection geojson格式数据
     */

  }, {
    key: "toGeojson",
    value: function toGeojson() {
      var featureCollection = {
        type: "FeatureCollection",
        features: []
      };
      if (this.entityObjArr.length == 0) return null;
      for (var i = 0; i < this.entityObjArr.length; i++) {
        var item = this.entityObjArr[i];
        var lnglats = item.getPositions(true); // geojson中 单个坐标 不含高度 否则geojsondatasourece加载会有问题

        var coordinates = [];

        for (var step = 0; step < lnglats.length; step++) {
          coordinates.push([lnglats[step][0], lnglats[step][1]]);
        }

        var style = item.getStyle();
        var geoType = this.transType(item.type);
        
        var feature = {
          "type": "Feature",
          "properties": {
            "plotType": item.type,
            "style": style,
            "arrowType": item.arrowType
          },
          "geometry": {
            "type": geoType,
            "coordinates": []
          }
        };

        switch (geoType) {
          case "Polygon":
            feature.geometry.coordinates = [coordinates];
            break;

          case "Point":
            feature.geometry.coordinates = coordinates;
            break;

          case "LineString":
            feature.geometry.coordinates = coordinates;
            break;
          case "arrow":
            feature.geometry.coordinates = coordinates;
            break;
        }

        feature.properties = Object.assign(feature.properties, item.properties);
        featureCollection.features.push(feature);
      }

      return featureCollection;
    } // 标绘类型和geojson数据类型相互转换

  }, {
    key: "transType",
    value: function transType(plotType) {
      var geoType = '';

      switch (plotType) {
        case "polyline":
          geoType = "LineString";
          break;

        case "polygon":
          geoType = "Polygon";
          break;

        case "point":
        case "gltfModel":
        case "label":
        case "billboard":
          geoType = "Point";
          break;

        default:
          geoType = plotType;
      }

      return geoType;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if(this.nowEditEntityObj && 'changeProperty' in this.nowEditEntityObj) {
        this.nowEditEntityObj.changeProperty()
      }
      // 取消当前绘制
      if (this.nowEditEntityObj) {
        this.nowEditEntityObj.destroy();
        this.nowEditEntityObj = null;
      }

      if (this.nowDrawEntityObj) {
        this.nowDrawEntityObj.destroy();
        this.nowDrawEntityObj = null;
      }

      for (var i = 0; i < this.entityObjArr.length; i++) {
        this.entityObjArr[i].destroy();
      }

      this.entityObjArr = [];
      this.nowEditEntityObj = null;

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.removeHandler) {
        this.removeHandler.destroy();
        this.removeHandler = null;
      }
    }
    /**
     * 移除某个绘制对象
     * @param {Object} entityObj 已绘制完成绘制对象
     */

  }, {
    key: "removeOne",
    value: function removeOne(entityObj) {
      if (!entityObj) return;
      if (!entityObj) return;

      if (entityObj.state != "endCreate" || entityObj.state != "endEdit") {
        entityObj.destroy();
      } else {
        this.removeByObjId(entityObj.objId);
      }
    }
    /**
     * 移除某个绘制对象
     * @param {Object} entityObj 已绘制完成绘制对象
     */

  }, {
    key: "remove",
    value: function remove(entityObj) {
      this.removeOne(entityObj);
    }
    /**
     * 移除全部绘制对象
     */

  }, {
    key: "removeAll",
    value: function removeAll() {
      // 取消当前绘制
      if (this.nowDrawEntityObj) {
        this.nowDrawEntityObj.destroy();
        this.nowDrawEntityObj = null;
      }

      if (this.nowEditEntityObj) {
        this.nowEditEntityObj.destroy();
        this.nowEditEntityObj = null;
      }

      for (var i = 0; i < this.entityObjArr.length; i++) {
        var obj = this.entityObjArr[i];
        obj.destroy();
      }

      this.entityObjArr = [];
      this.nowEditEntityObj = null;
    }
    /**
    * 是否包含某个对象
    * @param {Object} entityObj 绘制对象
    */

  }, {
    key: "hasEntityObj",
    value: function hasEntityObj(entityObj) {
      if (!entityObj) return false;
      var obj = this.getEntityObjByObjId(entityObj.objId);
      return obj != {} ? true : false;
    }
    /**
    * 根据id移除创建的对象
    * @param {String | Number} id 对象id
    */

  }, {
    key: "removeByObjId",
    value: function removeByObjId(id) {
      var obj = this.getEntityObjByObjId(id);
      this.entityObjArr.splice(obj.index, 1); // 触发on绑定的移除事件

      if (this.removeFun) this.removeFun(obj.entityObj, obj.entityObj?.getEntity());

      if (obj.entityObj) {
        obj.entityObj.destroy();
      }
    }
    /**
    * 根据attr.id移除创建的对象
    * @param {String | Number} id 创建时的attr.id
    */

  }, {
    key: "removeById",
    value: function removeById(id) {
      var obj = this.getEntityObjById(id);
      this.entityObjArr.splice(obj.index, 1); // 触发on绑定的移除事件

      if (this.removeFun) this.removeFun(obj.entityObj, obj.entityObj.getEntity());

      if (obj.entityObj) {
        obj.entityObj.destroy();
      }
    }
    /**
    * 根据id缩放至绘制的对象
    * @param {String} id 对象id
    */

  }, {
    key: "zoomToByObjId",
    value: function zoomToByObjId(id) {
      var obj = this.getEntityObjByObjId(id);

      if (obj.entityObj) {
        obj.entityObj.zoomTo();
      }
    }
    /**
     * 根据attr属性字段获取对象
     * @param {String} fieldName 属性字段名称
     * @param {String} [fieldValue] 属性值，若不填，则默认以id进行查询
     * @returns {Object} obj 对象在数组中位置以及对象
     */

  }, {
    key: "getEntityObjByField",
    value: function getEntityObjByField(fieldName, fieldValue) {
      var obj = {};

      if (!fieldValue) {
        // 如果缺少第二个参数 则默认以attr.id进行查询
        for (var i = 0; i < this.entityObjArr.length; i++) {
          var item = this.entityObjArr[i];

          if (item.attr.id == fieldName) {
            obj.entityObj = item;
            obj.index = i;
            break;
          }
        }
      } else {
        // 否则 以键值对的形式进行查询
        for (var ind = 0; ind < this.entityObjArr.length; ind++) {
          var _item = this.entityObjArr[ind];

          if (_item.attr[fieldName] == fieldValue) {
            obj.entityObj = _item;
            obj.index = ind;
            break;
          }
        }
      }

      return obj;
    }
    /**
     * 根据id设置对象的显示隐藏
     * @param {String | Number} id 对象id
     * @param {Boolean} visible 是否展示
     */

  }, {
    key: "setVisible",
    value: function setVisible(id, visible) {
      var obj = this.getEntityObjByField("id", id);
      if (obj.entityObj) obj.entityObj.setVisible(visible);
    }
    /**
     * 根据id获取对象
     * @param {String | Number} id entityObj的objid
     * @returns {Object} obj 对象在数组中位置以及对象
     */

  }, {
    key: "getEntityObjByObjId",
    value: function getEntityObjByObjId(id) {
      if (!id) return;
      var obj = {};

      for (var i = 0; i < this.entityObjArr.length; i++) {
        var item = this.entityObjArr[i];

        if (item.objId == id) {
          obj.entityObj = item;
          obj.index = i;
          break;
        }
      }

      return obj;
    }
    /**
     * 根据id获取对象，同getEntityObjByField('id',idvalue);
     * @param {String | Number} id 创建时的attr中的id
     * @returns {Object} obj 对象在数组中位置以及对象
     */

  }, {
    key: "getEntityObjById",
    value: function getEntityObjById(id) {
      if (!id) return;
      var obj = {};

      for (var i = 0; i < this.entityObjArr.length; i++) {
        var item = this.entityObjArr[i];

        if (item.attr.id == id) {
          obj.entityObj = item;
          obj.index = i;
          break;
        }
      }

      return obj;
    } // 绑定编辑

  }, {
    key: "bindEdit",
    value: function bindEdit() {
      var that = this; // 如果是线 面 则需要先选中

      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        if (!that.canEdit) return; // 若当前正在绘制 则无法进行编辑操作

        if (that.nowDrawEntityObj) return;
        var pick = that.viewer.scene.pick(evt.position);
        if (Cesium.defined(pick) && pick.id) {
          // 选中实体
          for (var i = 0; i < that.entityObjArr.length; i++) {
            if (pick.id.objId == that.entityObjArr[i].objId && (that.entityObjArr[i].state != "startCreate" || that.entityObjArr[i].state != "creating" || that.entityObjArr[i].state != "endEdit")) {
              // 结束上一个实体的编辑操作
              if (that.nowEditEntityObj) {
                if(!window.noLabelBool) return // 根据业务需求后加的，后期可删除
                if(that.nowEditEntityObj && 'changeProperty' in that.nowEditEntityObj) {
                  that.nowEditEntityObj.changeProperty()
                }
                
                that.nowEditEntityObj.endEdit();

                if (that.endEditFun) {
                  that.endEditFun(that.nowEditEntityObj, that.nowEditEntityObj.getEntity());
                }

                that.nowEditEntityObj = null;
              } // 开始当前实体的编辑
              if(that.entityObjArr[i] && 'changeProperty' in that.entityObjArr[i]) {
                that.entityObjArr[i].changeProperty(1)
              }
              that.entityObjArr[i].startEdit(function () {
                if (that.editingFun) that.editingFun(that.nowEditEntityObj, that.nowEditEntityObj.entity);
              });
              if (that.startEditFun) that.startEditFun(that.entityObjArr[i], pick.id); // 开始编辑

              that.nowEditEntityObj = that.entityObjArr[i];
              break;
            }
          }
        } else {
          // 未选中实体 则结束全部绘制
          if (that.nowEditEntityObj) {
            that.nowEditEntityObj.endEdit();
            if(that.nowEditEntityObj && 'changeProperty' in that.nowEditEntityObj) {
              that.nowEditEntityObj.changeProperty()
            }
            if (that.endEditFun) {
              that.endEditFun(that.nowEditEntityObj, that.nowEditEntityObj.getEntity());
            }

            that.nowEditEntityObj = undefined;
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
    } // 绑定右键删除

  }, {
    key: "bindRemove",
    value: function bindRemove() {
      var that = this; // 如果是线 面 则需要先选中

      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        if (!that.canRemove) return;
        /* console.log("that===>",that);*/

        if (!that.canEdit) return; // 若当前正在绘制 则无法进行删除

        if (that.nowDrawEntityObj) return;
        var pick = that.viewer.scene.pick(evt.position);
        if (!pick || !pick.id || !pick.id.objId) return;
        var entObj = that.getEntityObjByObjId(pick.id.objId).entityObj;
        if (!entObj) return;
        if (entObj.attr.plotId != that.plotId) return;
        /* let selectEntobj = undefined; */

        /* for (let i = 0; i < that.entityObjArr.length; i++) {
          if (pick.id.objId == that.entityObjArr[i].objId) {
            selectEntobj = that.entityObjArr[i];
            break;
          }
        } */

        that.createDelteDom(evt.position, pick.id.objId);
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
    }
  }, {
    key: "createDelteDom",
    value: function createDelteDom(px, objId) {
      if (!objId) return;
      var deleteDom = window.document.createElement("span");
      deleteDom.style.background = "rgba(0,0,0,0.5)";
      deleteDom.style.position = "absolute";
      deleteDom.style.color = "white";
      deleteDom.style.left = px.x + 10 + "px";
      deleteDom.style.top = px.y + 10 + "px";
      deleteDom.style.padding = "4px";
      deleteDom.style.cursor = "pointer";
      deleteDom.id = "vis3d-plot-delete";
      deleteDom.setAttribute("objId", objId);
      deleteDom.innerHTML = "\u5220\u9664";
      var mapDom = window.document.getElementById(this.viewer.container.id);
      mapDom.appendChild(deleteDom);
      var clsBtn = window.document.getElementById("vis3d-plot-delete");
      if (!clsBtn) return;
      var that = this;
      clsBtn.addEventListener("click", function (e) {
        var id = deleteDom.getAttribute("objId");
        that.removeByObjId(id);
      });
      document.addEventListener("click", function () {
        clsBtn.remove();
      });
    }
    /**
     * 结束编辑
     */

  }, {
    key: "endEdit",
    value: function endEdit() {
      if (this.nowEditEntityObj) {
        // 结束除当前选中实体的所有编辑操作
        if(this.nowEditEntityObj && 'changeProperty' in this.nowEditEntityObj) {
          this.nowEditEntityObj.changeProperty()
        }
        this.nowEditEntityObj.endEdit();

        if (this.endEditFun) {
          this.endEditFun(this.nowEditEntityObj, this.nowEditEntityObj.getEntity()); // 结束事件
        }

        this.nowEditEntityObj = null;
      }

      for (var i = 0; i < this.entityObjArr.length; i++) {
        this.entityObjArr[i].endEdit();
      }
    }
  }, {
    key: "done",
    value: function done() {
      if (this.nowEditEntityObj) {
        this.nowEditEntityObj.done();
        if (this.endEditFun) this.endEditFun(this.nowEditEntityObj, this.nowEditEntityObj.getEntity());
        this.nowEditEntityObj = undefined;
      }

      if (this.nowDrawEntityObj) {
        this.nowDrawEntityObj.done();
        this.entityObjArr.push(this.nowDrawEntityObj);
        if (this.endCreateFun) this.endCreateFun(this.nowDrawEntityObj, this.nowDrawEntityObj.getEntity());
        this.nowDrawEntityObj = undefined;
      }
    }
    /**
     * 获取当前所有对象
     * @returns {Array} entityObjArr
     */

  }, {
    key: "getEntityObjArr",
    value: function getEntityObjArr() {
      return this.entityObjArr;
    }
  }, {
    key: "createByType",
    value: function createByType(opt) {
      var entityObj = undefined;
      var name = "";
      opt = opt || {};
   
      if (opt.type == "polyline") {
        entityObj = new CreatePolyline(this.viewer, opt);
        name = "折线_";
      }

      if (opt.type == "polygon") {
        entityObj = new CreatePolygon(this.viewer, opt);
        name = "面_";
      }

      if (opt.type == "billboard") {
        entityObj = new CreateBillboard(this.viewer, opt);
        name = "图标_";
      }

      if (opt.type == "circle") {
        entityObj = new CreateCircle(this.viewer, opt);
        name = "圆_";
      }

      if (opt.type == "rectangle") {
        entityObj = new CreateRectangle(this.viewer, opt);
        name = "矩形_";
      }

      if (opt.type == "gltfModel") {
        entityObj = new CreateGltfModel(this.viewer, opt);
        name = "模型_";
      }

      if (opt.type == "point") {
        entityObj = new CreatePoint(this.viewer, opt);
        name = "点_";
      }

      if (opt.type == "label") {
        entityObj = new CreateLabel(this.viewer, opt);
        name = "文字_";
      }

      if (opt.type == "arrow") {
        /**
        * situationType值及对应的类型：
        *  	1-攻击箭头 2-攻击箭头（平尾）3-攻击箭头（燕尾）4-闭合曲面 5-钳击箭头 
        * 		6-单尖直箭头 7-粗单尖直箭头(带燕尾) 8-集结地 9-弓形面 10-直箭头 
        * 		11-矩形旗 12-扇形 13-三角旗 14-矩形波浪旗 17-多边形 18-圆形
        */
        if (!opt.arrowType) {
          console.log("缺少军事标绘类型");
          return;
        }

        entityObj = new CreateArrow(this.viewer, opt);
      }

      if (entityObj) entityObj.name = name + new Date().getTime();
      return entityObj;
    }
  }]);

  return DrawTool;
}();

/* 快捷绘制工具 */
var styleList = [{
  "name": "点",
  "type": "point",
  "styleType": "point"
}, {
  "name": "线",
  "type": "polyline",
  "styleType": "polyline",
  "style": {
    "clampToGround": true,
    "color": "#ffff00"
  }
}, {
  "name": "面",
  "type": "polygon",
  "styleType": "polygon",
  "style": {
    "color": "#0000ff",
    "outline": true,
    "outlineColor": "#ff0000",
    "heightReference": 1
  }
}, {
  "name": "圆形",
  "type": "circle",
  "styleType": "polygon",
  "style": {
    "color": "#0000ff",
    "colorAlpha": .3,
    "outline": true,
    "outlineColor": "#ff0000",
    "heightReference": 1
  }
}, {
  "name": "矩形",
  "type": "rectangle",
  "styleType": "polygon",
  "style": {
    "color": "#0000ff",
    "outline": true,
    "outlineColor": "#ff0000",
    "heightReference": 1
  }
}, {
  "name": "图标",
  "type": "billboard",
  "style": {
    "image": "./vis3d/images/plot/start.png",
    "heightReference": 1
  },
  "styleType": "billboard"
}, {
  "name": "文字",
  "type": "label",
  "style": {
    "text": "未命名",
    "fillColor": "#fff",
    "outline": false,
    "outlineWidth": 1,
    "outlineColor": "#ff0000",
    "heightReference": 0,
    "showBackground": true,
    "backgroundColor": "#000",
    "scale": 1
  },
  "styleType": "label"
}, {
  "name": "动态线",
  "type": "polyline",
  "styleType": "polyline",
  "style": {
    "clampToGround": true,
    "color": "#0EFCDC",
    "animateType": "flowline",
    "duration": 1000,
    "image": "./vis3d/images/texture/glow.png"
  }
}, {
  "name": "流动线",
  "type": "polyline",
  "styleType": "polyline",
  "style": {
    "clampToGround": true,
    "color": "#F9F507",
    "animateType": "flowline",
    "duration": 1000,
    "image": "./vis3d/images/texture/water.png"
  }
}];
var drawTool = undefined;
var pviewer = undefined;

var start = function start(opt, viewer) {
  var _styleList$filter$;

  if (!drawTool) {
    pviewer = viewer || window.viewer;
    drawTool = new DrawTool(pviewer, {
      canEdit: false
    });
  }

  var defaultStyle = ((_styleList$filter$ = styleList.filter(function (item) {
    return item.type == opt.type;
  })[0]) === null || _styleList$filter$ === void 0 ? void 0 : _styleList$filter$.style) || {};
  opt.style = Object.assign(defaultStyle, opt.style);
  return drawTool.start(opt);
};

var removeAll = function removeAll() {
  if (!drawTool) return;
  drawTool.removeAll();
};

var remove = function remove(entObj) {
  if (!entObj || !drawTool) return;
  drawTool.removeOne(entObj);
};

var draw = /*#__PURE__*/Object.freeze({
  __proto__: null,
  start: start,
  removeAll: removeAll,
  remove: remove
});

/**
 * 图层基类
 * @description 图层基类，一般不直接实例化
 * @alias BaseLayer
 * @class
 */

var BaseLayer = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置，可将
   * @param {String | Number} opt.id 地图服务id
   * @param {String} opt.url 地图服务地址
   * @param {Object} [opt.view] 图层定位视角
   * @param {Array} [opt.rectangle] 地图服务范围 [117,40,118,41]
   * @param {String} [opt.srs] 当前服务的坐标系 / 4326 3857
   * @param {Number|Function} [opt.alpha=1.0] 图层透明度
   * @param {Number|Function} [opt.nightAlpha=1.0] 在晚上的图层透明度
   * @param {Number|Function} [opt.dayAlpha=1.0] 在白天的图层透明度
   * @param {Number|Function} [opt.brightness=1.0] 图层亮度
   * @param {Number|Function} [opt.contrast=1.0] 图层对比度
   * @param {Number|Function} [opt.hue=0.0] 图层色调
   * @param {Number|Function} [opt.saturation=1.0] 图层饱和度
   * @param {Number|Function} [opt.gamma=1.0] 
   * @param {Boolean} [opt.show=true] 是否显示
   * @param {Number} [opt.maximumAnisotropy=maximum supported] 
   * @param {Number} [opt.minimumTerrainLevel] 显示该图层的最小地形层级
   * @param {Number} [opt.maximumTerrainLevel] 显示该图层的最大地形层级
   * @param {String} [opt.colorToAlpha] 颜色转透明度
   * @param {Number} [opt.colorToAlphaThreshold=0.004] 
   * 
   */
  function BaseLayer(viewer, opt) {
    _classCallCheck(this, BaseLayer);

    this.viewer = viewer;
    this.opt = opt || {}; // 定义imageryLayer基础参数种类

    var layerAttrs = ['alpha', 'nightAlpha', 'dayAlpha', 'brightness', 'contrast', 'hue', 'saturation', 'gamma', 'show', 'maximumAnisotropy', 'minimumTerrainLevel', 'maximumTerrainLevel', 'colorToAlpha', 'colorToAlphaThreshold'];
    /**
     * @property {String | Number} id 图层id
     */

    this.id = opt.id || Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));

    if (!opt.url && opt.type != "tdt" && opt.type != "grid" && opt.type != "tencent" && opt.type != "baidu") {
      console.log("缺少服务地址！", opt);
      return;
    }
    /**
     * @property {Object} providerAttr provider相关配置
     */


    this.providerAttr = {};
    /**
     * @property {Object} imageryLayerAttr imageryLayer相关配置
     */

    this.imageryLayerAttr = {};

    if (this.opt.rectangle) {
      var trectangle = new Cesium.Rectangle(Cesium.Math.toRadians(this.opt.rectangle[0]), Cesium.Math.toRadians(this.opt.rectangle[1]), Cesium.Math.toRadians(this.opt.rectangle[2]), Cesium.Math.toRadians(this.opt.rectangle[3]));
      this.providerAttr.rectangle = trectangle;
      this.imageryLayerAttr.rectangle = trectangle;
    }

    var tilingScheme = new Cesium.WebMercatorTilingScheme();

    if (opt.srs == "EPSG:4326" || opt.srs == "epsg:4326") {
      tilingScheme = new Cesium.GeographicTilingScheme({
        numberOfLevelZeroTilesX: opt.numberOfLevelZeroTilesX || 2,
        numberOfLevelZeroTilesY: opt.numberOfLevelZeroTilesY || 1
      });
    }

    this.providerAttr.tilingScheme = tilingScheme;
    this.providerAttr.url = opt.url; // 从opt中过滤出provider的参数

    var optFields = Object.keys(this.opt);

    for (var ind = 0; ind < optFields.length; ind++) {
      var field = optFields[ind];
      if (field == "rectangle") continue;

      if (layerAttrs.indexOf(field) == -1) {
        this.providerAttr[field] = this.opt[field];
      } else {
        this.imageryLayerAttr[field] = this.opt[field];
      }
    }
    /**
     * @property {Cesium.ImageryLayer} layer 图层
     */


    this._layer = null;
    /**
     * @property {Cesium.ImageryProvider} provider 图层
     */

    this._provider = {};
    /*  if (this.opt.srs == "EPSG:3857") {
        this.opt.tilingScheme = new Cesium.WebMercatorTilingScheme();
    } else if (this.opt.srs == "EPSG:4490") {
      } else if (this.opt.srs == "EPSG:4326") {
        this.opt.tilingScheme = new Cesium.GeographicTilingScheme();
    } else {
      } */
  }

  _createClass(BaseLayer, [{
    key: "layer",
    get: function get() {
      return this._layer;
    }
    /**
     * 加载
     */

  }, {
    key: "load",
    value: function load() {
      if (!this._provider || this._provider == {}) return;
      this._layer = new Cesium.ImageryLayer(this._provider, this.imageryLayerAttr);
      /* this.viewer.imageryLayers.add(this._layer, this.opt.index); */

      this.viewer.imageryLayers.add(this._layer);
      this._layer.attr = this.opt; // 保存配置信息
    }
  }, {
    key: "getLayer",
    value: function getLayer() {
      return this._layer;
    }
    /**
    * 移除
    */

  }, {
    key: "remove",
    value: function remove() {
      if (this._layer) this.viewer.imageryLayers.remove(this._layer);
    }
    /**
    * 展示
    */

  }, {
    key: "show",
    value: function show() {
      if (this._layer) {
        this._layer.show = true;
        this._layer.attr.show = true;
      }
    }
    /**
    * 隐藏
    */

  }, {
    key: "hide",
    value: function hide() {
      if (this._layer) {
        this._layer.show = false;
        this._layer.attr.show = false;
      }
    }
    /**
     * @param {Boolean} visible 是否显示
     */

  }, {
    key: "setVisible",
    value: function setVisible(visible) {
      visible = visible == undefined ? true : visible;

      if (visible) {
        this.show();
      } else {
        this.hide();
      }
    }
    /**
     * 缩放至图层
     */

  }, {
    key: "zoomTo",
    value: function zoomTo() {
      if (this.opt.view) {
        util$1.setCameraView(this.opt.view);
      } else {
        if (this._layer.type == "3dtiles") this.viewer.zoomTo(this._layer);
      }
    }
    /**
     * 设置透明度
     * @param {Number} alpha 透明度（0~1）
     */

  }, {
    key: "setAlpha",
    value: function setAlpha(alpha) {
      if (!this._layer) return;
      alpha = alpha == undefined ? 1 : alpha;
      this._layer.alpha = alpha;
    }
  }, {
    key: "lowerLayer",
    value: function lowerLayer() {
      if (this._layer) this.viewer.imageryLayers.lower(this._layer);
    }
  }, {
    key: "lowerLayerToBottom",
    value: function lowerLayerToBottom() {
      if (this._layer) this.viewer.imageryLayers.lowerToBottom(this._layer);
    }
  }, {
    key: "raiseLayer",
    value: function raiseLayer() {
      if (this._layer) this.viewer.imageryLayers.raise(this._layer);
    }
  }, {
    key: "raiselayerToTop",
    value: function raiselayerToTop() {
      if (this._layer) this.viewer.imageryLayers.raiseToTop(this._layer);
    }
  }]);

  return BaseLayer;
}();

/**
 * arcgis 切片类型图层（一般由arcmap切片后的数据发布）
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.ArcgiscacheLayer
 * @example 
 * let arcgisLayer = new vis3d.ArcgiscacheLayer(viewer,{
    url: "http://112.86.147.194:9009/data/demnewtile/L{arc_z}/R{arc_y}/C{arc_x}.png",
    minimumLevel: 1,
    maximumLevel: 19,
    minimumTerrainLevel: 1,
    view: {
        x: 118.73263653438936,
        y: 31.971959788539053,
        z: 6643.463555185671,
        heading: 341.6647257262609,
        pitch: -36.54290725763041,
        roll: 359.9323408763138,
    },
});
arcgisLayer.load();
 */

var ArcgiscacheLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(ArcgiscacheLayer, _BaseLayer);

  var _super = _createSuper(ArcgiscacheLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   * @param {String} opt.url 模型服务地址
   * @param {Number} opt.minimumLevel 地图服务最小层级
   * @param {Number} opt.maximumLevel 地图服务最大层级
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
   */
  function ArcgiscacheLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, ArcgiscacheLayer);

    _this = _super.call(this, viewer, opt);
    /**
     * @property {String} type 类型
     */

    _this.type = "arcgiscache";

    if (!Cesium.UrlTemplateImageryProvider.prototype.padLeft0) {
      Cesium.UrlTemplateImageryProvider.prototype.padLeft0 = function (numStr, n) {
        numStr = String(numStr);
        var len = numStr.length;

        while (len < n) {
          numStr = "0" + numStr;
          len++;
        }

        return numStr;
      };
    }

    var customTags = {
      //小写
      "arc_x": function arc_x(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0(x.toString(16), 8);
      },
      "arc_y": function arc_y(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0(y.toString(16), 8);
      },
      "arc_z": function arc_z(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0(level.toString(), 2);
      },
      "arc_z4490": function arc_z4490(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0((level + 1).toString(), 2);
      },
      //大写
      "arc_X": function arc_X(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0(x.toString(16), 8).toUpperCase();
      },
      "arc_Y": function arc_Y(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0(y.toString(16), 8).toUpperCase();
      },
      "arc_Z": function arc_Z(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0(level.toString(), 2).toUpperCase();
      },
      "arc_Z4490": function arc_Z4490(imageryProvider, x, y, level) {
        return imageryProvider.padLeft0((level + 1).toString(), 2).toUpperCase();
      }
    };
    var pattr = Object.assign(_this.providerAttr, {
      customTags: customTags
    });
    _this._provider = new Cesium.UrlTemplateImageryProvider(pattr);
    return _this;
  }

  return _createClass(ArcgiscacheLayer);
}(BaseLayer);

/**
 * mapserver 类型图层
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.MapserverLayer
 * @example 
 *      let mapserverLayer = new vis3d.MapserverLayer(viewer,{
 *              url: "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer",
 *              show: true,
 *      });
 *      mapserverLayer.load();
 */

var MapserverLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(MapserverLayer, _BaseLayer);

  var _super = _createSuper(MapserverLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置，其它参数见{@link BaseLayer}的Parameters。
   * @param {String} opt.url 地图服务地址
   * @param {String} [opt.token] 地图服务token
   * @param {String} [opt.layers] 服务中图层名称
   * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   * @param {Number} [opt.maximumLevel] 地图服务最大层级
   */
  function MapserverLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, MapserverLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "mapserver"; // 1.107后 使用fromUrl

    _this._provider = Cesium.ArcGisMapServerImageryProvider.fromUrl(_this.providerAttr.url, _this.providerAttr);
    return _this;
  } // 重写加载


  _createClass(MapserverLayer, [{
    key: "load",
    value: function load() {
      if (!this._provider || this._provider == {}) return;
      this._layer = Cesium.ImageryLayer.fromProviderAsync(this._provider, this.imageryLayerAttr);
      /* this.viewer.imageryLayers.add(this._layer, this.opt.index); */

      this.viewer.imageryLayers.add(this._layer);
      this._layer.attr = this.opt; // 保存配置信息
    }
  }]);

  return MapserverLayer;
}(BaseLayer);

/**
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.GridLayer
 * @example 
 *      let gridLayer = new vis3d.GridLayer(viewer,{
                show: true,
                glowColor : "#FF0000",
                alpha: 1,
        });
        gridLayer.load();
 */

var GridLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(GridLayer, _BaseLayer);

  var _super = _createSuper(GridLayer);

  /**
  * @param {Cesium.Viewer} viewer 地图viewer对象 
  * @param {Object} opt 基础配置，其它参数见{@link BaseLayer}的Parameters。
  * @param {cells} [opt.cells=4] 网格单元的数量
  * @param {String} [opt.color='#33FFFF'] 网格线颜色
  * @param {String} [opt.glowColor='#33FFFF'] 网格线发光色
  * @param {Number} [opt.glowWidth=3] 网格线发光宽度
  * @param {String} [opt.backgroundColor='#CCCCCC'] 背景色
  * @param {Number} [opt.tileWidth=256] 服务切片宽度
  * @param {Number} [opt.tileHeight=256] 服务切片高度
  * @param {Number} [opt.canvasSize==256] 渲染的canvas尺寸
  */
  function GridLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, GridLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "grid";
    var color = Cesium.Color.fromCssColorString(opt.color || '#33FFFF');
    var glowColor = Cesium.Color.fromCssColorString(opt.glowColor || '#33FFFF');
    var backgroundColor = Cesium.Color.fromCssColorString(opt.backgroundColor || '#CCCCCC');
    _this.providerAttr.cells = opt.cells || 4;
    _this.providerAttr.glowWidth = opt.glowWidth || 3;
    _this.providerAttr.color = color;
    _this.providerAttr.glowColor = glowColor;
    _this.providerAttr.backgroundColor = backgroundColor;
    _this._provider = new Cesium.GridImageryProvider(_this.providerAttr);
    return _this;
  }

  return _createClass(GridLayer);
}(BaseLayer);

// 用于设置各个entity的样式
var defaulteStyle = {
  point: {
    color: "#00FFFF",
    colorAlpha: 1,
    outlineWidth: 1,
    outlineColor: "#00FFFF",
    outlineColorAlpha: 1,
    pixelSize: 20,
    heightReference: 1,
    disableDepthTestDistance: Number.MAX_VALUE
  },
  polyline: {
    color: "#FFFF00",
    colorAlpha: 1,
    width: 3,
    clampToGround: 1
  },
  polygon: {
    heightReference: 1,
    fill: true,
    color: "#00FFFF",
    colorAlpha: 1,
    outline: true,
    outlineWidth: 1,
    outlineColor: "#FFFF00",
    outlineColorAlpha: 1
  },
  label: {
    fillColor: "#ffffff",
    fillColorAlpah: 1,
    disableDepthTestDistance: Number.MAX_VALUE,
    heightReference: 1,
    showBackground: false,
    backgroundColor: "#000000",
    backgroundColorAlpha: 0.5,
    outlineColor: "#C6A300",
    outlineColorAlpha: 1.0,
    outlineWidth: 1.0,
    pixelOffset: [0, -20],
    font: "14px",
    verticalOrigin: 1,
    // {CENTER: 0, BOTTOM: 1, BASELINE: 2, TOP: -1}
    HorizontalOrigin: 0 // {CENTER: 0, LEFT: 1, RIGHT: -1}

  }
};
/**
 * 常见颜色转为Cesium中颜色
 * @param {Cesium.Color | String} color
 * @param {Number} alpha
 * @returns {Cesium.Color}
 */

var transColor = function transColor(color, alpha) {
  var tcolor = undefined;
  alpha = alpha || 1.0;

  if (color instanceof Cesium.Color) {
    tcolor = color.clone();
  } else {
    tcolor = Cesium.Color.fromCssColorString(color).withAlpha(alpha);
  }

  return tcolor;
};
/**
 * 数组转cartsian2
 * @param {Array} arr
 * @returns {Cesium.Cartesian2}
 */


var transC2 = function transC2(arr) {
  arr = arr || [0, 0];
  return new Cesium.Cartesian2(arr[0], arr[1]);
};

var setPointStyle = function setPointStyle(ent, style) {
  if (!ent) return;
  var point = ent.point;
  style = Object.assign(defaulteStyle.point, style);
  style.color = transColor(style.color, style.colorAlpha);
  style.outlineColor = transColor(style.outlineColor, style.outlineColorAlpha);

  for (var i in style) {
    point[i] = style[i];
  }
};

var setLabelStyle = function setLabelStyle(ent, style) {
  if (!ent) return;
  var label = ent.label;
  style = Object.assign(defaulteStyle.label, style);
  style.fillColor = style.color || style.fillColor;
  style.fillColor = transColor(style.fillColor, style.fillColorAlpah);
  style.outlineColor = transColor(style.outlineColor, style.outlineColorAlpha);

  if (style.showBackground) {
    style.backgroundColor = transColor(style.backgroundColor, style.backgroundColorAlpha);
  }

  style.pixelOffset = transC2(style.pixelOffset);

  for (var i in style) {
    label[i] = style[i];
  }
};

var setPolylineStyle = function setPolylineStyle(ent, style) {
  if (!ent) return;
  var polyline = ent.polyline;
  style = Object.assign(defaulteStyle.polyline, style);
  style.color = transColor(style.color, style.colorAlpha);
  style.material = style.color.clone();

  for (var i in style) {
    polyline[i] = style[i];
  }
};

var setPolygonStyle = function setPolygonStyle(ent, style) {
  if (!ent) return;
  var polygon = ent.polygon;
  style = Object.assign(defaulteStyle.polygon, style);
  style.color = transColor(style.color, style.colorAlpha);
  style.material = style.color.clone();
  if (style.heightReference == 1) style.height = undefined;

  for (var i in style) {
    // 如果配置fill属性的话 会卡死
    polygon[i] = style[i];
  }
}; // =================================== 矩形 ===================================

/**
 * geojson/wfs类型图层数据加载
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.GeojsonLayer
 * @example
 * let geojsonLayer = new vis3d.GeojsonLayer(viewer,{
 *  show: false,
    url: "data/area.json",
    alpha: 0.5,
    style: {
    point: {
        color: "#00FFFF",

        colorAlpha: 1,
        outlineWidth: 1,
        outlineColor: "#000000",
        outlineColorAlpha: 1,
        pixelSize: 20,
    },
    polyline: {
        color: "#FFFF00",
        colorAlpha: 1,
        width: 3,
        clampToGround: 1,
    },
    polygon: {
        heightReference: 1,
        fill: true,
        color: {
            conditions: "random",
            type: "color", // 随机数返回值类型 number / color(16进制颜色)
        },
        "color": {  // 支持多种方式赋值
                        "field": "name",
                        "conditions": [
                            ['${name} >= "东部战区"', '#000000'],
                            ['true', 'color("blue")']
                        ]
                    }, 
        "color":{
            "field" : "name",
            "conditions" : "random" , // 可不填 
             }
                            
        colorAlpha: 1,
        outline: true,
        outlineWidth: 1,
        outlineColor: "#FFFF00",
        outlineColorAlpha: 1,
        },
    },
     tooltip: [
    {
        field: "name",
        fieldName: "名称",
    },
    {
        field: "ADCODE99",
        fieldName: "编号",
    },
    ];
  geojsonLayer.load();
 */

var GeojsonLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(GeojsonLayer, _BaseLayer);

  var _super = _createSuper(GeojsonLayer);

  /**
   *
   * @param {Cesium.viewer} viewer 地图viewer对象
   * @param {Object} opt 基础配置
   * @param {String} opt.url 模型服务地址
   * @param {Object} [opt.style] 要素样式
   * @param {typeName} [opt.typeName] 图层名称
   */
  function GeojsonLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, GeojsonLayer);

    _this = _super.call(this, viewer, opt);
    /**
     * @property {String} type 类型
     */

    _this.type = "geojson";
    _this.viewer = viewer;
    _this.opt = opt || {};
    var defaultStyleVal = {
      point: {
        color: "#00FFFF",
        colorAlpha: 1,
        outlineWidth: 1,
        outlineColor: "#00FFFF",
        outlineColorAlpha: 1,
        pixelSize: 14,
        heightReference: 1,
        disableDepthTestDistance: Number.MAX_VALUE
      },
      polyline: {
        color: "#FFFF00",
        colorAlpha: 1,
        width: 3,
        clampToGround: 1
      },
      polygon: {
        heightReference: 1,
        fill: true,
        color: "#00FFFF",
        colorAlpha: 1,
        outline: true,
        outlineWidth: 1,
        outlineColor: "#FFFF00",
        outlineColorAlpha: 1
      },
      label: {
        text: '--',
        fillColor: "#ffffff",
        disableDepthTestDistance: Number.MAX_VALUE,
        heightReference: 1,
        showBackground: false,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        outlineColor: "#00FFFF",
        outlineWidth: 1.0
      }
    };
    /**
     * @property {Object} style 要素样式
     */

    _this.featureStyle = opt.style;

    for (var i in opt.style) {
      var feastyle = opt.style[i];
      var defaultStyle = defaultStyleVal[i];
      _this.featureStyle[i] = Object.assign(defaultStyle, feastyle);
    }

    _this.url = _this.opt.url || "";
    return _this;
  }
  /**
   *
   * @callback loadcallback
   * @param {number} responseCode
   * @param {string} responseMessage
   */

  /**
   * 图层加载
   * @param {loadcallback} fun 加载完成后的回调函数
   */


  _createClass(GeojsonLayer, [{
    key: "load",
    value: function load(fun) {
      var _this2 = this;

      var that = this;
      var dataSource = this.viewer.dataSources.add(Cesium.GeoJsonDataSource.load(this.url));
      dataSource.then(function (ds) {
        that._layer = ds;
        that._layer.attr = that.opt;

        for (var i = that._layer.entities.values.length - 1; i >= 0; i--) {
          var ent = that._layer.entities.values[i]; // 获取geojson中的属性

          var properties = ent.properties.getValue(that.viewer.clock.currentTime);

          if (ent.billboard) {
            // geojson中的point会模型转为billboard 转为point
            var position = ent.position.getValue(that.viewer.clock.currentTime);

            if (that.featureStyle.point) {
              var pointStyle = that.featureStyle["point"];
              var newPointStyle = JSON.parse(JSON.stringify(pointStyle)); // 数据隔离

              newPointStyle.color = that.getColorByProperty(pointStyle.color, properties);

              var _ent = that._layer.entities.add({
                position: position,
                point: {}
              });

              setPointStyle(_ent, newPointStyle);
              if (that.opt.tooltip) that.bindTooltip(_ent, that.opt.tooltip, properties);
            }

            if (_this2.featureStyle.label) {
              var labelStyle = that.featureStyle['label'];
              labelStyle = JSON.parse(JSON.stringify(labelStyle)); // 数据隔离

              labelStyle.text = _this2.featureStyle.label.text.field ? properties[_this2.featureStyle.label.text.field] : _this2.featureStyle.label.text;

              var _ent2 = that._layer.entities.add({
                position: position,
                label: {
                  text: "----"
                }
              });

              setLabelStyle(_ent2, labelStyle);
            }

            that._layer.entities.remove(ent); // 此处删去原来的billboard

          } else {

            if (ent.polyline) {
              var lineStyle = that.featureStyle["polyline"];
              var newLineStyle = JSON.parse(JSON.stringify(lineStyle));
              newLineStyle.color = that.getColorByProperty(lineStyle.color, properties);
              setPolylineStyle(ent, newLineStyle);
            } else if (ent.polygon) {
              var polygonStyle = that.featureStyle["polygon"];
              var newPolygonStyle = JSON.parse(JSON.stringify(polygonStyle));
              newPolygonStyle.color = that.getColorByProperty(polygonStyle.color, properties);
              setPolygonStyle(ent, newPolygonStyle);
            }

            if (that.opt.tooltip) that.bindTooltip(ent, that.opt.tooltip, properties);
          }
        }

        if (that.opt.flyTo) that.zoomTo();
        if (fun) fun();
      });
    } // entity绑定弹窗

  }, {
    key: "bindTooltip",
    value: function bindTooltip(ent, tooltipObj, properties) {
      if (!ent) return;
      var content = '';

      if (typeof tooltipObj == 'string') {
        content = tooltipObj;
      } else if (tooltipObj instanceof Array) {
        tooltipObj.forEach(function (item) {
          var field = item.field,
              fieldName = item.fieldName;
          content += "".concat(fieldName, "\uFF1A").concat(properties[field], "<br/>");
        });
      }

      ent.tooltip = content;
    } // 根据geojson的属性来设置颜色

  }, {
    key: "getColorByProperty",
    value: function getColorByProperty(colorObj, properties) {
      var color = "";
      colorObj = colorObj || "#ff0000";

      if (colorObj == "random") {
        color = this.getRandomColor();
      } else if (typeof colorObj == "string") {
        color = colorObj;
      } else {
        var field = colorObj.field; // "field": "name",
        // "conditions": [
        //     ['${name} >= "东部战区"', '#000000'],
        //     ['true', 'color("blue")']
        // ]

        if (colorObj.conditions instanceof Array) {
          color = this.getConditionValue(field, properties[field], colorObj.conditions);
        } else {
          color = this.getRandomColor();
        }
      }

      return color;
    }
  }, {
    key: "getConditionValue",
    value: function getConditionValue(key, value, conditions) {
      var styleValue = null; // 获取默认值

      for (var ind = 0; ind < conditions.length; ind++) {
        if (conditions[ind][0] == "true") {
          styleValue = conditions[ind][1];
          break;
        }
      }

      for (var i = 0; i < conditions.length; i++) {
        var condition = conditions[i];
        var replaceStr = "${" + key + "}";
        var str = condition[0].replace(replaceStr, '"' + value + '"');

        if (eval(str)) {
          styleValue = condition[1];
          break;
        }
      }

      return styleValue;
    }
  }, {
    key: "getEntityField",
    value: function getEntityField(field, value) {

      for (var i = 0; i < this._layer.entities.length; i++) {
        var ent = this._layer.entities[i];
        var properties = ent.properties.getValue(that.viewer.clock.currentTime);

        if (properties[field] == value) {
          break;
        }
      }

      return;
    }
    /**
     * 缩放至图层
     */

  }, {
    key: "zoomTo",
    value: function zoomTo() {
      if (!this._layer) return;

      if (this._layer.attr.view) {
        util$1.setCameraView(opt.view);
      } else {
        this.viewer.zoomTo(this._layer);
      }
    }
    /**
     * 移除
     */

  }, {
    key: "remove",
    value: function remove() {
      if (this._layer) {
        this.viewer.dataSources.remove(this._layer);
      }
    }
    /**
     * 展示
     */

  }, {
    key: "show",
    value: function show() {
      if (this._layer) {
        this._layer.show = true;
        this._layer.attr.show = true;
      }
    }
    /**
     * 隐藏
     */

  }, {
    key: "hide",
    value: function hide() {
      if (this._layer) {
        this._layer.attr.show = false;
        this._layer.show = false;
      }
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this._layer) {
        this._layer.entities.removeAll();

        this.viewer.dataSources.remove(this._layer);
      }
    }
  }, {
    key: "getContent",
    value: function getContent(properties, fields) {
      var html = "";

      for (var i = 0; i < fields.length; i++) {
        var _fields$i = fields[i],
            field = _fields$i.field,
            fieldName = _fields$i.fieldName;
        var value = properties[field];
        html += "\n                <tr>\n                    <td>".concat(fieldName, "\uFF1A</td>\n                    <td>").concat(value, "</td>\n                </tr>\n            ");
      }

      return "\n            <table>".concat(html, "</table>\n        ");
    }
  }, {
    key: "getRandomColor",
    value: function getRandomColor() {
      var color = "#";
      var arr = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F"];

      for (var i = 0; i < 6; i++) {
        var num = parseInt(Math.random() * 16);
        color += arr[num];
      }

      return color;
    }
  }]);

  return GeojsonLayer;
}(BaseLayer);

/**
 * 天地图在线服务加载
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.TDTLayer
 * 
 */

var TDTLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(TDTLayer, _BaseLayer);

  var _super = _createSuper(TDTLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   * @param {String | String[]} [opt.keys] 天地图服务密钥（需在天地图官网申请）
   * @param {String} layerName 天地图图层名称（vec（矢量底图）/ cva（矢量注记）/ img（影像底图）/ ter（地形晕渲）cta（地形注记）/ ibo（全球境界）/ eva（矢量英文注记）/ eia（影像英文注记））
   * @param {String | Number} crs 坐标系EPSG（4326/3857）
   * @param {Number} opt.minimumLevel 地图服务最小层级
   * @param {Number} opt.maximumLevel 地图服务最大层级
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
  */
  function TDTLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, TDTLayer);

    // 内置keys
    var keys = ["313cd4b28ed520472e8b43de00b2de56", "83b36ded6b43b9bc81fbf617c40b83b5", "0ebd57f93a114d146a954da4ecae1e67", "6c99c7793f41fccc4bd595b03711913e", "56b81006f361f6406d0e940d2f89a39c"];
    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "tdt";
    _this.opt = opt || {};

    if (!_this.opt.keys || _this.opt.keys.length == 0) {
      var random = Math.random() * keys.length;
      random = Math.floor(random);
      _this.key = keys[random];
    } else {
      if (Array.isArray(_this.opt.keys)) {
        var _random = Math.random() * _this.opt.key.length;

        _random = Math.floor(_random);
        _this.key = keys[_random];
      } else {
        _this.key = _this.opt.keys;
      }
    } // vec（矢量底图）/ cva（矢量注记）/ img（影像底图）/ ter（地形晕渲）
    // cta（地形注记）/ ibo（全球境界）/ eva（矢量英文注记）/ eia（影像英文注记）


    if (!_this.opt.layerName) {
      console.log("缺少图层名称");
      return _possibleConstructorReturn(_this);
    }

    var tileMatrixSetID = "";
    var tdtLayerName = "";

    if (_this.opt.crs == 4326) {
      // 经纬度
      tileMatrixSetID = "c";
      tdtLayerName = _this.opt.layerName + "_c";
    } else {
      // 墨卡托  3857
      tileMatrixSetID = "w";
      tdtLayerName = _this.opt.layerName + "_w";
    }

    var url = 'https://t{s}.tianditu.gov.cn/' + tdtLayerName + '/wmts?service=WMTS&version=1.0.0&request=GetTile&tilematrix={TileMatrix}&layer=' + _this.opt.layerName + '&style={style}&tilerow={TileRow}&tilecol={TileCol}&tilematrixset={TileMatrixSet}&format=tiles&tk=' + _this.key;
    var maxLevel = 18;
    var tileMatrixLabels = [];

    for (var z = 0; z <= maxLevel; z++) {
      tileMatrixLabels[z] = z.toString();
    }

    var pattr = {
      url: url,
      layer: tdtLayerName,
      style: 'default',
      format: 'tiles',
      tileMatrixSetID: tileMatrixSetID,
      subdomains: ['0', '1', '2', '3', '4', '5', '6', '7'],
      tileMatrixLabels: tileMatrixLabels,
      tilingScheme: new Cesium.WebMercatorTilingScheme()
    };
    pattr = Object.assign(_this.providerAttr || {}, pattr);
    _this._provider = new Cesium.WebMapTileServiceImageryProvider(pattr);
    return _this;
  }

  return _createClass(TDTLayer);
}(BaseLayer);

/**
 * 单张图片图层（一般由arcmap切片后的数据发布）
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.SingleImageLayer
 * @example 
 * let singleLayer = new vis3d.SingleImageLayer(viewer,{
    url: "./vis3d/images/layer/world.jpg",
    rectangle: [-180, -90, 180, 90],
    minimumLevel: 1,
    maximumLevel: 19,
    view: {
        x: 118.73263653438936,
        y: 31.971959788539053,
        z: 6643.463555185671,
        heading: 341.6647257262609,
        pitch: -36.54290725763041,
        roll: 359.9323408763138,
    },
});
singleLayer.load();
 */

var SingleImageLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(SingleImageLayer, _BaseLayer);

  var _super = _createSuper(SingleImageLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   * @param {Array} opt.rectangle 地图服务范围 [117,40,118,41]
   */
  function SingleImageLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, SingleImageLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "singleImage";
    _this.providerAttr.tileWidth = _this.providerAttr.tileWidth || 256;
    _this.providerAttr.tileHeight = _this.providerAttr.tileWidth || 256;
    _this._provider = new Cesium.SingleTileImageryProvider(_this.providerAttr);
    return _this;
  }

  return _createClass(SingleImageLayer);
}(BaseLayer);

/**
 * tms类型图层（一般由arcmap切片后的数据发布）
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.TMSLayer
 * @example 
 * let tmsLayer = new vis3d.TMSLayer(viewer,{
    url: "",
    minimumLevel: 1,
    maximumLevel: 19,
    minimumTerrainLevel: 1,
    view: {
        x: 118.73263653438936,
        y: 31.971959788539053,
        z: 6643.463555185671,
        heading: 341.6647257262609,
        pitch: -36.54290725763041,
        roll: 359.9323408763138,
    },
});
tmsLayer.load();
 */

var TMSLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(TMSLayer, _BaseLayer);

  var _super = _createSuper(TMSLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   * @param {String} opt.url 模型服务地址
   * @param {Number} opt.minimumLevel 地图服务最小层级
   * @param {Number} opt.maximumLevel 地图服务最大层级
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   */
  function TMSLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, TMSLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "tms";
    _this._provider = new Cesium.TileMapServiceImageryProvider(_this.opt);
    return _this;
  }

  return _createClass(TMSLayer);
}(BaseLayer);

/**
 * xyz切片类型图层
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.XYZLayer
 */

var XYZLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(XYZLayer, _BaseLayer);

  var _super = _createSuper(XYZLayer);

  /**
  * @param {Cesium.Viewer} viewer 地图viewer对象 
  * @param {Object} opt 基础配置
  * @param {String} opt.url 模型服务地址
  * @param {Number} opt.minimumLevel 地图服务最小层级
  * @param {Number} opt.maximumLevel 地图服务最大层级
  * @param {Number} [opt.tileWidth=256] 服务切片宽度
  * @param {Number} [opt.tileHeight=256] 服务切片高度
  * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
  */
  function XYZLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, XYZLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "xyz";
    _this._provider = new Cesium.UrlTemplateImageryProvider(_this.providerAttr);
    return _this;
  } // 获取当前图层


  _createClass(XYZLayer, [{
    key: "layer",
    get: function get() {
      return this._layer;
    }
  }, {
    key: "provider",
    get: function get() {
      return this._provider;
    }
  }]);

  return XYZLayer;
}(BaseLayer);

var TilesetEdit = /*#__PURE__*/function () {
  function TilesetEdit(viewer, opt) {
    _classCallCheck(this, TilesetEdit);

    this.viewer = viewer;
    this.opt = opt || {};
    this.tileset = opt.tileset;

    if (!opt.tileset) {
      console.log("缺少模型！");
      return;
    }

    this.initTransform = this.tileset._root.transform.clone();
    this.initTransform_inverse = Cesium.Matrix4.inverse(this.initTransform.clone(), new Cesium.Matrix4());
    var centerMtx = Cesium.Transforms.eastNorthUpToFixedFrame(this.tileset.boundingSphere.center.clone());
    this.centerMtx_inverse = Cesium.Matrix4.inverse(centerMtx.clone(), new Cesium.Matrix4());
    this.newCenter = Cesium.Matrix4.multiplyByPoint(this.centerMtx_inverse.clone(), this.tileset.boundingSphere.center.clone(), new Cesium.Cartesian3());
    this.rotate = {
      xMatrix: undefined,
      yMatrix: undefined,
      zMatrix: undefined
    };
    this.scale = {
      xMatrix: undefined,
      yMatrix: undefined,
      zMatrix: undefined
    };
    this.hprMatrix = undefined;
    this.translation = undefined;
    this.centerPosition = this.tileset.boundingSphere.center.clone();
  } // 计算平移矩阵


  _createClass(TilesetEdit, [{
    key: "setPosition",
    value: function setPosition(position) {
      if (!(position instanceof Cesium.Cartesian3)) {
        position = Cesium.Cartesian3.fromDegrees(Number(position[0]), Number(position[1]), Number(position[2] || 0));
      }

      this.centerPosition = position.clone(); // 局部坐标系下坐标

      var newPosition = Cesium.Matrix4.multiplyByPoint(this.centerMtx_inverse.clone(), position.clone(), new Cesium.Cartesian3()); // 当前相对起点的偏移量

      var translation = Cesium.Cartesian3.subtract(newPosition.clone(), this.newCenter.clone(), new Cesium.Cartesian3());
      this.translation = translation.clone();
      this.update();
    }
  }, {
    key: "update",
    value: function update() {
      // 还原
      this.tileset._root.transform = this.initTransform.clone(); // 1、平移

      if (this.translation) {
        var nowMovelMtx = Cesium.Matrix4.fromTranslation(this.translation.clone(), new Cesium.Matrix4());
        Cesium.Matrix4.multiply(this.tileset._root.transform, nowMovelMtx.clone(), this.tileset._root.transform);
      } // 2、旋转


      if (this.rotate.xMatrix) {
        var nowRotationMtx = Cesium.Matrix4.fromRotation(this.rotate.xMatrix, new Cesium.Matrix4());
        Cesium.Matrix4.multiply(this.tileset._root.transform, nowRotationMtx.clone(), this.tileset._root.transform);
      }

      if (this.rotate.yMatrix) {
        var _nowRotationMtx = Cesium.Matrix4.fromRotation(this.rotate.yMatrix, new Cesium.Matrix4());

        Cesium.Matrix4.multiply(this.tileset._root.transform, _nowRotationMtx.clone(), this.tileset._root.transform);
      }

      if (this.rotate.zMatrix) {
        var nowRotationMtx2 = Cesium.Matrix4.fromRotation(this.rotate.zMatrix, new Cesium.Matrix4());
        Cesium.Matrix4.multiply(this.tileset._root.transform, nowRotationMtx2.clone(), this.tileset._root.transform);
      }

      if (this.hprMatrix) {
        Cesium.Matrix4.multiply(this.tileset._root.transform, this.hprMatrix.clone(), this.tileset._root.transform);
      } // 3、缩放


      if (this.scale.xMatrix) {
        Cesium.Matrix4.multiply(this.tileset._root.transform, this.scale.xMatrix.clone(), this.tileset._root.transform);
      }

      if (this.scale.yMatrix) {
        Cesium.Matrix4.multiply(this.tileset._root.transform, this.scale.yMatrix.clone(), this.tileset._root.transform);
      }

      if (this.scale.zMatrix) {
        Cesium.Matrix4.multiply(this.tileset._root.transform, this.scale.zMatrix.clone(), this.tileset._root.transform);
      }
    } // 根据hpr来设置模型姿态
    // setHPR(opt) {
    //   const { heading, pitch, roll } = opt || {}
    //   let center = this.centerPosition.clone();
    //   const hpr = new Cesium.HeadingPitchRoll(
    //     Cesium.Math.toRadians(heading || 0),
    //     Cesium.Math.toRadians(pitch || 0),
    //     Cesium.Math.toRadians(roll || 0)
    //   );
    //   const quaternion = Cesium.Transforms.headingPitchRollQuaternion(center, hpr);
    //   const mtx3 = Cesium.Matrix3.fromQuaternion(quaternion, new Cesium.Matrix3());
    //   this.hprMatrix = Cesium.Matrix4.fromRotation(mtx3, new Cesium.Matrix4())
    //   this.update();
    // }
    // 计算缩放矩阵

  }, {
    key: "setScale",
    value: function setScale(scale) {
      this.setScaleX(scale);
      this.setScaleY(scale);
      this.setScaleZ(scale);
    }
  }, {
    key: "setScaleX",
    value: function setScaleX(sc) {
      var scale = new Cesium.Cartesian3(sc, 1, 1);
      var nowScaleMtx = Cesium.Matrix4.fromScale(scale.clone(), new Cesium.Matrix4());
      this.scale.xMatrix = nowScaleMtx.clone();
      this.update();
    }
  }, {
    key: "setScaleY",
    value: function setScaleY(sc) {
      var scale = new Cesium.Cartesian3(1, sc, 1);
      var nowScaleMtx = Cesium.Matrix4.fromScale(scale.clone(), new Cesium.Matrix4());
      this.scale.yMatrix = nowScaleMtx.clone();
      this.update();
    }
  }, {
    key: "setScaleZ",
    value: function setScaleZ(sc) {
      var scale = new Cesium.Cartesian3(1, 1, sc);
      var nowScaleMtx = Cesium.Matrix4.fromScale(scale.clone(), new Cesium.Matrix4());
      this.scale.zMatrix = nowScaleMtx.clone();
      this.update();
    } // 计算旋转矩阵

  }, {
    key: "setRotateX",
    value: function setRotateX(angle) {
      console.log("edit x==>", angle);
      var rotation = Cesium.Matrix3.fromRotationX(Cesium.Math.toRadians(angle));
      this.rotate.xMatrix = rotation.clone();
      this.update();
    }
  }, {
    key: "setRotateY",
    value: function setRotateY(angle) {
      console.log("edit y==>", angle);
      var rotation = Cesium.Matrix3.fromRotationY(Cesium.Math.toRadians(angle));
      this.rotate.yMatrix = rotation.clone();
      this.update();
    }
  }, {
    key: "setRotateZ",
    value: function setRotateZ(angle) {
      var rotation = Cesium.Matrix3.fromRotationZ(Cesium.Math.toRadians(angle));
      console.log("edit z==>", angle);
      this.rotate.zMatrix = rotation.clone();
      this.update();
    }
  }, {
    key: "reset",
    value: function reset() {
      this.tileset._root.transform = this.initTransform.clone();
      this.rotate = {
        xMatrix: undefined,
        yMatrix: undefined,
        zMatrix: undefined
      };
      this.scale = {
        xMatrix: undefined,
        yMatrix: undefined,
        zMatrix: undefined
      };
      this.hprMatrix = undefined;
      this.translation = undefined;
    }
  }]);

  return TilesetEdit;
}();

var TilesetLayer = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象
   * @param {Object} opt 基础配置，其余参数同{@link Cesium.Cesium3DTileset}
   * @param {String} opt.url 模型服务地址
   * @param {Boolean} [opt.show=true] 是否显示
   * @param {Object} [opt.view] 模型定位视角
   * @param {Object} [opt.center] 设置模型中心点,如果只修改高度，x、y可不设置
   * @param {Cesium.Cartesian3 | Object} [opt.position] 模型位置，和center二选一
   * @param {Object} [style] 模型样式
   * @example
   * let tilesetLayer = new vis3d.TilesetLayer(viewer,{
      url: "http://192.168.21.108:9999/jcjy/jcjy-hb/tileset.json",
      show: true,
      maximumScreenSpaceError: 16,
      maximumMemoryUsage: 1024,
      center: {
          z: 10
      },
      style: {
          color: {
              conditions: [
                  ['${Height} >= 100', 'color("purple", 0.5)'],
                  ['${Height} >= 50', 'color("red")'],
                  ['true', 'color("blue")']
              ]
          },
          show: '${Height} > 0'
      },
      view: {
          "x": 109.7884118470029,
          "y": 39.590384952017764,
          "z": 1565.7899788867958,
          "heading": 331.1978494043747,
          "pitch": -8.45296669256617,
          "roll": 0.00043210090111595544,
          "duration": 0
      }
  });
  tilesetLayer.load();
   */
  function TilesetLayer(viewer, opt) {
    _classCallCheck(this, TilesetLayer);

    /* super(viewer, opt); */
    this.viewer = viewer;
    this.opt = opt || {};
    /**
    * @property {String} type 类型
    */

    this.type = "3dtiles";

    if (!this.opt.url) {
      console.log("缺少服务地址！", opt);
    }

    this._layer = undefined;
    this.tilesetEdit = undefined;
  } // 获取当前图层


  _createClass(TilesetLayer, [{
    key: "layer",
    get: function get() {
      return this._layer;
    } // 加载

  }, {
    key: "load",
    value: function () {
      var _load = _asyncToGenerator( /*#__PURE__*/_regeneratorRuntime().mark(function _callee(fun) {
        var defaultVal, tilesetAttr, tileset;
        return _regeneratorRuntime().wrap(function _callee$(_context) {
          while (1) {
            switch (_context.prev = _context.next) {
              case 0:
                defaultVal = {
                  maximumScreenSpaceError: 16,
                  skipLevelOfDetail: true,
                  preferLeaves: true,
                  maximumMemoryUsage: 512
                };
                tilesetAttr = Object.assign(defaultVal, this.opt);
                _context.next = 4;
                return Cesium.Cesium3DTileset.fromUrl(tilesetAttr.url, tilesetAttr);

              case 4:
                tileset = _context.sent;
                this.viewer.scene.primitives.add(tileset);
                if (!this.tilesetEdit) this.tilesetEdit = new TilesetEdit(this.viewer, {
                  tileset: tileset
                });
                this._layer = tileset;
                this._layer.layerConfig = this.opt; // 保存配置信息

                this._layer.initBoundingSphere = tileset.boundingSphere.clone(); // 初始化中心

                this._layer.show = this.opt.show == undefined ? true : this.opt.show;
                if (this.opt.center) this.setCenter(this.opt.center);
                if (this.opt.orientation) this.setOrientation(this.opt.orientation);
                if (this.opt.scale) this.setScale(this.opt.scale);
                if (this.opt.flyTo) this.zoomTo();
                if (this.opt.style) this.updateStyle(this.opt.style);
                if (fun) fun(tileset);
                return _context.abrupt("return", tileset);

              case 18:
              case "end":
                return _context.stop();
            }
          }
        }, _callee, this);
      }));

      function load(_x) {
        return _load.apply(this, arguments);
      }

      return load;
    }()
    /**
     * 销毁模型
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this._layer) this.viewer.scene.primitives.remove(this._layer);
      this._layer = undefined;
    }
    /**
     * 移除模型
     */

  }, {
    key: "remove",
    value: function remove() {
      if (this._layer) this.viewer.scene.primitives.remove(this._layer);
    }
    /**
     * 显示模型
     */

  }, {
    key: "show",
    value: function show() {
      if (this._layer) {
        this._layer.show = true;
        this._layer.layerConfig.show = true;
        if (this.opt.style) this.updateStyle(this.opt.style); // 显示时 要重置样式
      }
    }
    /**
     * 隐藏模型
     */

  }, {
    key: "hide",
    value: function hide() {
      if (this._layer) {
        this._layer.show = false;
        this._layer.layerConfig.show = false;
      }
    }
    /**
     * 显示隐藏
     */

  }, {
    key: "setVisible",
    value: function setVisible(visible) {
      if (visible) this.show();else this.hide();
    }
    /**
     * 定位至模型
     */

  }, {
    key: "zoomTo",
    value: function zoomTo() {
      if (!this._layer) return;

      if (this._layer.layerConfig.view) {
        util$1.setCameraView(this.opt.view);
      } else {
        this.viewer.flyTo(this._layer, new Cesium.HeadingPitchRange(Cesium.Math.toRadians(0), Cesium.Math.toRadians(-60), this._layer.boundingSphere.radius * 5));
      }
    }
    /**
     * 设置模型中心点
     * @param {Object|Cesium.Cartesian3} opt 
     * @param {Number} opt.x 经度
     * @param {Number} opt.y 纬度
     * @param {Number} opt.z 高度
     */

  }, {
    key: "setCenter",
    value: function setCenter(opt) {
      opt = opt || {};

      if (opt instanceof Cesium.Cartesian3) {
        this.tilesetEdit.setPosition(opt.clone());
      } else {
        var origin = Cesium.Cartographic.fromCartesian(this._layer.boundingSphere.center);
        opt.x = opt.x || Cesium.Math.toDegrees(origin.longitude);
        opt.y = opt.y || Cesium.Math.toDegrees(origin.latitude);
        this.tilesetEdit.setPosition([opt.x, opt.y, opt.z]);
      }
    }
    /**
     * 设置模型姿态
     * @param {Object|Cesium.Cartesian3} opt 
     * @param {Number} opt.heading 偏转角
     * @param {Number} opt.pitch 仰附角
     * @param {Number} opt.roll 翻滚角
     */

  }, {
    key: "setOrientation",
    value: function setOrientation(opt) {
      if (!opt) return;

      var _ref = opt || {},
          heading = _ref.heading,
          pitch = _ref.pitch,
          roll = _ref.roll;

      if (heading != undefined) this.tilesetEdit.setRotateZ(heading);
      if (pitch != undefined) this.tilesetEdit.setRotateY(pitch);
      if (roll != undefined) this.tilesetEdit.setRotateX(roll);
    }
  }, {
    key: "setScale",
    value: function setScale(opt) {
      if (!opt) return;

      if (isNaN(opt)) {
        var _ref2 = opt || {},
            x = _ref2.x,
            y = _ref2.y,
            z = _ref2.z;

        if (x != undefined) this.tilesetEdit.setScaleX(x);
        if (y != undefined) this.tilesetEdit.setScaleY(y);
        if (z != undefined) this.tilesetEdit.setScaleZ(z);
      } else {
        this.tilesetEdit.setScale(Number(opt));
      }
    }
    /**
     * 修改模型样式
     * @param {Object} style 
     * @example
     *  style={
        color : {
            conditions : [
                ['${Height} >= 100', 'color("purple", 0.5)'],
                ['${Height} >= 50', 'color("red")'],
                ['true', 'color("blue")']
            ]
        },
        show : '${Height} > 0'
     * }
     */

  }, {
    key: "updateStyle",
    value: function updateStyle(style) {
      if (!style) return;
      this._layer.style = new Cesium.Cesium3DTileStyle(style);
    }
    /**
     * 设置模型透明度
     * @param {Number} [alpha=1] 
     */

  }, {
    key: "setAlpha",
    value: function setAlpha(alpha) {
      alpha = alpha == undefined ? 1 : alpha;
      this._layer.style = new Cesium.Cesium3DTileStyle({
        color: "color('rgba(255,255,255," + alpha + ")')"
      });
    }
  }]);

  return TilesetLayer;
}();

/**
 * 加载OGC标准的wms服务
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.WMSLayer
 * @example 
 * let wmsLayer = new vis3d.WMSLayer(viewer,{
    url : 'http://localhost:8080/geoserver/wms',
    layers : 'xian:satellite16', 
    parameters: {
        service : 'WMS',
        format: 'image/png',
        transparent: true,
    },
    minimumLevel: 1,
    maximumLevel: 19,
    view: {
        x: 118.73263653438936,
        y: 31.971959788539053,
        z: 6643.463555185671,
        heading: 341.6647257262609,
        pitch: -36.54290725763041,
        roll: 359.9323408763138,
    },
});
wmsLayer.load();
 */

var WMSLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(WMSLayer, _BaseLayer);

  var _super = _createSuper(WMSLayer);

  /**
  * @param {Cesium.Viewer} viewer 地图viewer对象 
  * @param {Object} opt 基础配置
  * @param {String} opt.url 模型服务地址
  * @param {String} opt.layers wms服务中图层名称
  * @param {Object} opt.parameters 地图获取功能GetMap所需要的参数
  * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
  * @param {String} [opt.crs] CRS定义，WMS版本 >= 1.3.0
  * @param {String} [opt.srs] SRS定义，WMS版本为1.1.0 或 1.1.1
  * @param {Number} opt.minimumLevel 地图服务最小层级
  * @param {Number} opt.maximumLevel 地图服务最大层级
  * @param {Number} [opt.tileWidth=256] 服务切片宽度
  * @param {Number} [opt.tileHeight=256] 服务切片高度
  */
  function WMSLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, WMSLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "wms";

    if (!_this.providerAttr.layers) {
      console.log("当前服务缺少 layers 参数！", _this.providerAttr);
    }

    _this._provider = new Cesium.WebMapServiceImageryProvider(_this.providerAttr);
    return _this;
  }

  return _createClass(WMSLayer);
}(BaseLayer);

/**
 * 加载OGC标准的wmts服务
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.WMTSLayer
 * @example 
 * let wmtsLayer = new vis3d.WMTSLayer(viewer,{
    url : 'http://localhost:8080/geoserver/wms',
    layers : 'xian:satellite16', 
    parameters: {
        service : 'WMS',
        format: 'image/png',
        transparent: true,
    },
    minimumLevel: 1,
    maximumLevel: 19,
    view: {
        x: 118.73263653438936,
        y: 31.971959788539053,
        z: 6643.463555185671,
        heading: 341.6647257262609,
        pitch: -36.54290725763041,
        roll: 359.9323408763138,
    },
});
wmtsLayer.load();
 */

var WMTSLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(WMTSLayer, _BaseLayer);

  var _super = _createSuper(WMTSLayer);

  /**
  * @param {Cesium.Viewer} viewer 地图viewer对象 
  * @param {Object} opt 基础配置
  * @param {String} opt.url 模型服务地址
  * @param {String} opt.layer 服务中图层名称
  * @param {String} opt.style 样式设置
  * @param {String} opt.tileMatrixSetID 
  * @param {Array} opt.tileMatrixLabels 
  * @param {String} [opt.format='image/jpeg'] 切片类型
  * @param {Number} opt.minimumLevel 地图服务最小层级
  * @param {Number} opt.maximumLevel 地图服务最大层级
  * @param {Number} [opt.tileWidth=256] 服务切片宽度
  * @param {Number} [opt.tileHeight=256] 服务切片高度
  */
  function WMTSLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, WMTSLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "wmts";

    if (!_this.providerAttr.layers) {
      console.log("当前服务缺少 layers 参数！", _this.providerAttr);
    }

    _this._provider = new Cesium.WebMapTileServiceImageryProvider(_this.providerAttr);
    return _this;
  }

  return _createClass(WMTSLayer);
}(BaseLayer);

/**
 * mapserver 类型图层
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.UrlTemplateLayer
 * @example 

 */

var UrltemplateLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(UrltemplateLayer, _BaseLayer);

  var _super = _createSuper(UrltemplateLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置，其它参数见{@link BaseLayer}的Parameters。
   * @param {String} opt.url 地图服务地址
   * @param {String} [opt.token] 地图服务token
   * @param {String} [opt.layers] 服务中图层名称
   * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   * @param {Number} [opt.maximumLevel] 地图服务最大层级
   */
  function UrltemplateLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, UrltemplateLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "mapserver";
    _this._provider = new Cesium.UrlTemplateImageryProvider(_this.providerAttr);
    return _this;
  }

  return _createClass(UrltemplateLayer);
}(BaseLayer);

function BaiduImageryProvider(options) {
  this._errorEvent = new Cesium.Event();
  this._tileWidth = 256;
  this._tileHeight = 256;
  this._maximumLevel = 18;
  this._minimumLevel = 1;
  var southwestInMeters = new Cesium.Cartesian2(-33554054, -33746824);
  var northeastInMeters = new Cesium.Cartesian2(33554054, 33746824);
  this._tilingScheme = new Cesium.WebMercatorTilingScheme({
    rectangleSouthwestInMeters: southwestInMeters,
    rectangleNortheastInMeters: northeastInMeters
  });
  this._rectangle = this._tilingScheme.rectangle;
  var resource = Cesium.Resource.createIfNeeded(options.url);
  this._resource = resource;
  this._tileDiscardPolicy = undefined;
  this._credit = undefined;
  this._readyPromise = undefined;
}

Object.defineProperties(BaiduImageryProvider.prototype, {
  url: {
    get: function get() {
      return this._resource.url;
    }
  },
  proxy: {
    get: function get() {
      return this._resource.proxy;
    }
  },
  tileWidth: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('tileWidth must not be called before the imagery provider is ready.');
      }

      return this._tileWidth;
    }
  },
  tileHeight: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('tileHeight must not be called before the imagery provider is ready.');
      }

      return this._tileHeight;
    }
  },
  maximumLevel: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('maximumLevel must not be called before the imagery provider is ready.');
      }

      return this._maximumLevel;
    }
  },
  minimumLevel: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('minimumLevel must not be called before the imagery provider is ready.');
      }

      return this._minimumLevel;
    }
  },
  tilingScheme: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('tilingScheme must not be called before the imagery provider is ready.');
      }

      return this._tilingScheme;
    }
  },
  tileDiscardPolicy: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('tileDiscardPolicy must not be called before the imagery provider is ready.');
      }

      return this._tileDiscardPolicy;
    }
  },
  rectangle: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('rectangle must not be called before the imagery provider is ready.');
      }

      return this._rectangle;
    }
  },
  errorEvent: {
    get: function get() {
      return this._errorEvent;
    }
  },
  ready: {
    get: function get() {
      return this._resource;
    }
  },
  readyPromise: {
    get: function get() {
      return this._readyPromise;
    }
  },
  credit: {
    get: function get() {
      if (!this.ready) {
        throw new Cesium.DeveloperError('credit must not be called before the imagery provider is ready.');
      }

      return this._credit;
    }
  }
});

BaiduImageryProvider.prototype.requestImage = function (x, y, level, request) {
  var r = this._tilingScheme.getNumberOfXTilesAtLevel(level);

  var c = this._tilingScheme.getNumberOfYTilesAtLevel(level);

  var s = this.url.replace("{x}", x - r / 2).replace("{y}", c / 2 - y - 1).replace("{z}", level).replace("{s}", Math.floor(10 * Math.random()));
  return Cesium.ImageryProvider.loadImage(this, s);
};
/**
 * mapserver 类型图层
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.BaiduLayer
 */


var BaiduLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(BaiduLayer, _BaseLayer);

  var _super = _createSuper(BaiduLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置，其它参数见{@link BaseLayer}的Parameters。
   * @param {String} opt.url 地图服务地址
   * @param {String} [opt.token] 地图服务token
   * @param {String} [opt.layers] 服务中图层名称
   * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   * @param {Number} [opt.maximumLevel] 地图服务最大层级
   */
  function BaiduLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, BaiduLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "baidu";
    var pattr = Object.assign(_this.providerAttr || {}, {
      url: "http://online{s}.map.bdimg.com/onlinelabel/?qt=tile&x={x}&y={y}&z={z}&styles=pl&scaler=1&p=1"
    });
    _this._provider = new BaiduImageryProvider(pattr);
    return _this;
  }

  return _createClass(BaiduLayer);
}(BaseLayer);

/**
 * mapserver 类型图层
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.MapserverLayer
 * @example 

 */

var TencentLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(TencentLayer, _BaseLayer);

  var _super = _createSuper(TencentLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置，其它参数见{@link BaseLayer}的Parameters。
   * @param {String} opt.url 地图服务地址
   * @param {String} [opt.token] 地图服务token
   * @param {String} [opt.layers] 服务中图层名称
   * @param {Boolean} [opt.enablePickFeatures=true] 是否可通过鼠标拾取元素
   * @param {Number} [opt.tileWidth=256] 服务切片宽度
   * @param {Number} [opt.tileHeight=256] 服务切片高度
   * @param {Number} [opt.maximumLevel] 地图服务最大层级
   */
  function TencentLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, TencentLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "tencent";

    var lyrurl = _this.getUrlByType(opt.layerType || "1");

    var pattr = {
      url: lyrurl,
      customTags: {
        sx: function sx(imageryProvider, x, y, level) {
          return x >> 4;
        },
        sy: function sy(imageryProvider, x, y, level) {
          return (1 << level) - y >> 4;
        }
      }
    };
    pattr = Object.assign(_this.providerAttr || {}, pattr);
    _this._provider = new Cesium.UrlTemplateImageryProvider(pattr);
    return _this;
  }

  _createClass(TencentLayer, [{
    key: "getUrlByType",
    value: function getUrlByType(type) {
      var url = "";

      switch (type) {
        case "1":
          // 影像图
          url = "https://p2.map.gtimg.com/sateTiles/{z}/{sx}/{sy}/{x}_{reverseY}.jpg?version=400";
          break;

        case "2":
          // 矢量图
          url = "https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={reverseY}&styleid=1&version=297";
          break;

        case "3":
          // 黑色风格
          url = "https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={reverseY}&styleid=4&scene=0";
          break;

        case "4":
          // 注记图1
          url = "https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={reverseY}&styleid=3&scene=0";
          break;

        case "5":
          // 注记图2
          url = "https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={reverseY}&styleid=2&version=297";
          break;
      }

      return url;
    }
  }]);

  return TencentLayer;
}(BaseLayer);

/**
 * 单张图片图层（一般由arcmap切片后的数据发布）
 * @class
 * @augments BaseLayer
 * @alias BaseLayer.OSMLayer
 * @example 

 */

var OSMLayer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(OSMLayer, _BaseLayer);

  var _super = _createSuper(OSMLayer);

  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   * @param {Array} opt.rectangle 地图服务范围 [117,40,118,41]
   */
  function OSMLayer(viewer, opt) {
    var _this;

    _classCallCheck(this, OSMLayer);

    _this = _super.call(this, viewer, opt);
    /**
    * @property {String} type 类型
    */

    _this.type = "osm"; // url: 'https://tile-{s}.openstreetmap.fr/hot/{z}/{x}/{y}.png' 标准
    // url:  "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png" 黑色

    var pattr = Object.assign(_this.providerAttr || {}, {
      subdomains: ['a', 'b', 'c', 'd']
    });
    _this._provider = new Cesium.UrlTemplateImageryProvider(pattr);
    return _this;
  }

  return _createClass(OSMLayer);
}(BaseLayer);

/**
 * 图层控制类
 * @description 图层控制类，通过此类对象，可直接添加相关类型图层，并对添加的图层对象进行控制，而不用多次new 不同类型的图层对象。
 * @class
 */

var LayerTool = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 当前viewer对象 
   * @param {Object} [opt] 其他参数
   */
  function LayerTool(viewer, opt) {
    _classCallCheck(this, LayerTool);

    this.viewer = viewer;
    /**
     * @property {Array} layerObjs 图层对象数组
     */

    this._layerObjs = [];
  }

  _createClass(LayerTool, [{
    key: "layerObjs",
    get: function get() {
      return this._layerObjs;
    }
    /**
     * 新增图层
     * @param {Object} opt 图层属性
     * @param {String | Number} [opt.id] 图层id，如果不传入，则自动生成
     * @param {String} opt.type 图层的类别（xyz、wfs、geojson、mapserver、arcgiscache、tdt、singleImage、tms、3dtiles、wms、grid）
     * @param {String} opt.alpha 图层的透明度
     * @returns {Object} 图层对象
     */

  }, {
    key: "add",
    value: function add(opt) {
      var layerObj = null;
      var success = opt.success;
      opt = JSON.parse(JSON.stringify(opt || {}));
      var type = opt.type; // 自动设置图层的index

      switch (type) {
        case "xyz":
          //xyz格式切片
          layerObj = new XYZLayer(this.viewer, opt);
          break;

        case "wfs": // wfs服务

        case "geojson":
          // geojson格式数据
          layerObj = new GeojsonLayer(this.viewer, opt);
          break;

        case "mapserver":
          // arcgis标准mapserver服务
          layerObj = new MapserverLayer(this.viewer, opt);
          break;

        case "arcgiscache":
          // arcmap的wgs84切片
          layerObj = new ArcgiscacheLayer(this.viewer, opt);
          break;

        case "tdt":
          // 天地图图层
          layerObj = new TDTLayer(this.viewer, opt);
          break;

        case "singleImage":
          // 单张图片  
          layerObj = new SingleImageLayer(this.viewer, opt);
          break;

        case "tms":
          // 标准tms类型
          layerObj = new TMSLayer(this.viewer, opt);
          break;

        case "3dtiles":
          // 模型
          layerObj = new TilesetLayer(this.viewer, opt);
          break;

        case "wms":
          // ogc wms服务
          layerObj = new WMSLayer(this.viewer, opt);
          break;

        case "wmts":
          // ogc wmts服务
          layerObj = new WMTSLayer(this.viewer, opt);
          break;

        case "grid":
          // 网格图层
          layerObj = new GridLayer(this.viewer, opt);
          break;

        case "tencent":
          // 腾讯地图
          layerObj = new TencentLayer(this.viewer, opt);
          break;

        case "baidu":
          // 百度地图
          layerObj = new BaiduLayer(this.viewer, opt);
          break;

        case "osm":
          // osm
          layerObj = new OSMLayer(this.viewer, opt);
          break;

        case "urltemplate":
          layerObj = new UrltemplateLayer(this.viewer, opt);
          break;
      }

      if (!layerObj) return;

      if (layerObj.type == "3dtiles" || layerObj.type == "geojson") {
        layerObj.load(function (layer) {
          // 当为3dtiles时 setAlpha和success里的设置样式可能会冲突
          if (opt.alpha != undefined) layerObj.setAlpha(opt.alpha);
          layerObj.setVisible(opt.show == undefined ? true : opt.show);
          if (success) success(layerObj, layer);
        });
      } else {
        layerObj.load();
        if (opt.alpha != undefined) layerObj.setAlpha(opt.alpha);
        layerObj.setVisible(opt.show == undefined ? true : opt.show);
        if (success) success(layerObj, layerObj.layer);
      }

      this._layerObjs.push(layerObj);

      opt.id = opt.id || Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
      opt.alpha = opt.alpha == undefined ? 1 : opt.alpha;
      layerObj.attr = opt; // 绑定属性文件 与mapConfig.js进行关联

      return layerObj;
    }
    /**
     * 根据id获取当前图层对象
     * @param {String | Number} id 
     * @returns {Object} layerObj为图层对象，index为图层对象在数组中位置
     */

  }, {
    key: "getLayerObjById",
    value: function getLayerObjById(id) {
      if (!id) return;
      var obj = {};

      for (var i = 0; i < this._layerObjs.length; i++) {
        if (this._layerObjs[i].attr.id == id) {
          obj = {
            layerObj: this._layerObjs[i],
            index: i
          };
          break;
        }
      }

      return obj;
    }
    /**
     * 根据Objid获取当前图层对象
     * @param {String | Number} id 
     * @returns {Object} layerObj为图层对象，index为图层对象在数组中位置
     */

  }, {
    key: "getLayerObjByObjId",
    value: function getLayerObjByObjId(id) {
      if (!id) return;
      var obj = {};

      for (var i = 0; i < this._layerObjs.length; i++) {
        if (this._layerObjs[i].objId == id) {
          obj = {
            layerObj: this._layerObjs[i],
            index: i
          };
          break;
        }
      }

      return obj;
    }
    /**
     * 获取当前图层对象
     * @param {Object} query 
     */

    /* getLayerObj(query) {
        let { key, value } = query;
        let obj = {};
        for (let i = 0; i < this._layerObjs.length; i++) {
            if (this._layerObjs[i].attr[key] == value) {
                obj = {
                    layerObj: this._layerObjs[i],
                    index: i
                }
                break;
            }
        }
    } */

    /**
     * 移除图层对象
     * @param {Object} layerObj 图层对象
     */

  }, {
    key: "removeLayerObj",
    value: function removeLayerObj(layerObj) {
      if (!layerObj) return;
      this.removeLayerObjById(layerObj.id);
    }
    /**
     * 根据id移除图层对象
     * @param {String | Number} id 图层对象id
    */

  }, {
    key: "removeLayerObjById",
    value: function removeLayerObjById(id) {
      if (!id) return;
      var lyropt = this.getLayerObjById(id);

      if (lyropt && lyropt.layerObj) {
        lyropt.layerObj.remove();

        this._layerObjs.splice(lyropt.index, 1);
      }
    }
    /**
     * 移除所有图层对象
     */

  }, {
    key: "removeAll",
    value: function removeAll() {
      for (var i = 0; i < this._layerObjs.length; i++) {
        this._layerObjs[i].remove();
      }

      this._layerObjs = [];
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      this.removeAll();
      this._layerObjs = [];
      delete this._layerObjs;
    }
    /**
     * 根据id隐藏图层
     * @param {String | Number} id 图层对象id
     */

  }, {
    key: "hideById",
    value: function hideById(id) {
      if (!id) return;
      var layerOpt = this.getLayerObjById(id);

      if (layerOpt && layerOpt.layerObj) {
        layerOpt.layerObj.hide();
        layerOpt.layerObj.attr.show = false;
      }
    }
    /**
     * 根据id显示图层
     * @param {String | Number} id 图层对象id
     */

  }, {
    key: "showById",
    value: function showById(id) {
      if (!id) return;
      var layerOpt = this.getLayerObjById(id);

      if (layerOpt && layerOpt.layerObj) {
        layerOpt.layerObj.show();
        layerOpt.layerObj.attr.show = true;
      }
    }
    /**
     * 根据id设置图层显示隐藏
     * @param {String | Number} id 图层对象id
     * @param {Boolean} isShow 是否显示 
     */

  }, {
    key: "setVisible",
    value: function setVisible(id, isShow) {
      if (!id) return;

      if (isShow) {
        this.showById(id);
      } else {
        this.hideById(id);
      }
    }
    /**
     * 根据图层对象id，缩放到某个图层
     * @param {String | Number} id 图层对象id
    */

  }, {
    key: "zoomTo",
    value: function zoomTo(id) {
      if (!id) return;
      var layobj = this.getLayerObjById(id) || {};
      if (layobj && layobj.layerObj) layobj.layerObj.zoomTo();
    }
    /**
     * 隐藏所有图层
    */

  }, {
    key: "hideAll",
    value: function hideAll() {
      for (var i = 0; i < this._layerObjs.length; i++) {
        this._layerObjs[i].hide();
      }
    }
    /**
     * 获取当前所有显示的图层
     * @returns {Array} 图层对象数组
    */

  }, {
    key: "getAllshow",
    value: function getAllshow() {
      var arr = [];

      for (var i = 0; i < this._layerObjs.length; i++) {
        if (this._layerObjs[i].attr.show) {
          arr.push(this._layerObjs[i]);
        }
      }

      return arr;
    }
    /**
     * 获取当前所有隐藏的图层
     * @returns {Array} 图层对象数组
    */

  }, {
    key: "getAllhide",
    value: function getAllhide() {
      var arr = [];

      for (var i = 0; i < this._layerObjs.length; i++) {
        if (!this._layerObjs[i].attr.show) {
          arr.push(this._layerObjs[i]);
        }
      }

      return arr;
    }
    /**
     * 根据图层属性字段来进行查询
     * @param {String} field 字段名称
     * @param {String} val 字段值
     * @returns {Array} 符合查询条件的图层对象数组
    */

  }, {
    key: "getLayerObjByField",
    value: function getLayerObjByField(field, val) {
      if (!field) return;
      var returnData = [];

      for (var i = 0; i < this._layerObjs.length; i++) {
        if (this._layerObjs[i].attr[field] == val) {
          returnData.push(this._layerObjs[i]);
        }
      }

      return returnData;
    }
    /* lowerLayer(opt) {
        if (!opt) return;
        if (opt instanceof String) {
            opt = {
                key: "id",
                value: opt
            }
        }
        let obj = this.getLayerObj(opt);
        if (obj && obj.layerObj) obj.layerObj.lowerLayer()
    }
    lowerLayerToBottom(opt) {
        if (!opt) return;
        if (opt instanceof String) {
            opt = {
                key: "id",
                value: opt
            }
        }
        let obj = this.getLayerObj(opt);
        if (obj && obj.layerObj) obj.layerObj.lowerLayerToBottom()
    }
    raiseLayer() {
        if (!opt) return;
        if (opt instanceof String) {
            opt = {
                key: "id",
                value: opt
            }
        }
        let obj = this.getLayerObj(opt);
        if (obj && obj.layerObj) obj.layerObj.raiseLayer()
    }
    raiselayerToTop() {
        if (!opt) return;
        if (opt instanceof String) {
            opt = {
                key: "id",
                value: opt
            }
        }
        let obj = this.getLayerObj(opt);
        if (obj && obj.layerObj) obj.layerObj.raiselayerToTop()
    } */

  }]);

  return LayerTool;
}();

/**
 * 加载图层方法
 */
var layerTool = undefined;
/**
 * 新增图层
 * @param {Object} opt 
 * @param {Cesium.Viewer} viewer 地图对象 
 * @returns {Object} 图层obj对象
 */

var add = function add(opt, viewer) {
  if (!layerTool) {
    layerTool = new LayerTool(viewer);
  }

  return layerTool.add(opt);
};
/**
 * 移除单个图层
 * @param {*} layerObj 图层obj对象
 * @returns 
 */


var remove$1 = function remove(layerObj) {
  if (!layerObj || !layerTool) return;
  layerTool.removeLayerObj(layerObj);
};
/**
 * 移除全部
 */


var removeAll$1 = function removeAll() {
  if (!layerObj) return;
  layerTool.removeAll();
};

var layerload = /*#__PURE__*/Object.freeze({
  __proto__: null,
  add: add,
  remove: remove$1,
  removeAll: removeAll$1
});

/**
 * 量算基类
 * @description 量算基类，一般不直接实例化，而实例化其子类（见下方Classes）
 * @class
 * @alias BaseMeasure
 */

var BaseMeasure = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象
   * @param {Object} opt 基础配置 
   */
  function BaseMeasure(viewer, opt) {
    _classCallCheck(this, BaseMeasure);

    this.viewer = viewer;
    this.opt = opt || {};
    /**
     * @property {String} objId 唯一标识id
     */

    this.objId = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    this.objId = "m" + this.objId
    /**
     * @property {String} state 标识当前状态 no startCreate creating endCreate startEdit endEdit editing
     */

    this.state = null;
    this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this.floatLable = null;
    /**
     * 
     * @property {String} unit 单位
     */

    this.unit = opt.unit;
    this.controlPoints = [];
    this.pointStyle = {};
    this.modifyPoint = null;
    /**
     * @property {Object} promptStyle 鼠标弹窗样式
     */

    this.promptStyle = opt.prompt || {
      show: true,
      offset: {
        x: 20,
        y: 20
      }
    };
    this.scale = this.opt.scale || [1, 1];
  }

  _createClass(BaseMeasure, [{
    key: "endCreate",
    value: function endCreate() {}
    /**
     * 结束当前操作 包括编辑和绘制
     */

  }, {
    key: "done",
    value: function done() {}
  }, {
    key: "createLine",
    value: function createLine(positions, clampToGround) {
      if (!positions) return;
      var ent = this.viewer.entities.add({
        polyline: {
          positions: new Cesium.CallbackProperty(function () {
            return positions;
          }, false),
          show: true,
          material: new Cesium.PolylineOutlineMaterialProperty({
            color: Cesium.Color.GOLD,
            outlineWidth: 1,
            outlineColor: Cesium.Color.BLACK
          }),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          width: 3,
          clampToGround: clampToGround,
          arcType: clampToGround ? Cesium.ArcType.GEODESIC : Cesium.ArcType.NONE,
        }
      });
      return ent;
    } // 操作控制

  }, {
    key: "forbidDrawWorld",
    value: function forbidDrawWorld(isForbid) {
      this.viewer.scene.screenSpaceCameraController.enableRotate = !isForbid;
      this.viewer.scene.screenSpaceCameraController.enableTilt = !isForbid;
      this.viewer.scene.screenSpaceCameraController.enableTranslate = !isForbid;
      this.viewer.scene.screenSpaceCameraController.enableInputs = !isForbid;
    }
  }, {
    key: "createLabel",
    value: function createLabel(c, text) {
      if (!c) return;
      return this.viewer.entities.add({
        position: c,
        label: {
          text: text || "",
          font: '18px Helvetica',
          fillColor: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cesium.Cartesian2(0, -20)
        }
      });
    }
    /**
     * 设置单位
     * @param {String} unit 
     */

  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      if (!unit) return;
      this.unit = unit;
    } // 角度计算

  }, {
    key: "getAzimuthtAndCenter",
    value: function getAzimuthtAndCenter(mtx, positions) {
      if (!positions || positions.length < 2) return;
      var center = positions[0].clone();
      mtx = mtx || Cesium.Transforms.eastNorthUpToFixedFrame(center.clone());
      var mtxInverse = Cesium.Matrix4.inverse(mtx, new Cesium.Matrix4());
      var aim = positions[1].clone();
      center = Cesium.Matrix4.multiplyByPoint(mtxInverse, center, new Cesium.Cartesian3());
      aim = Cesium.Matrix4.multiplyByPoint(mtxInverse, aim, new Cesium.Cartesian3());
      var newC = Cesium.Cartesian3.subtract(aim, center, new Cesium.Cartesian3());
      newC = Cesium.Cartesian3.normalize(newC, new Cesium.Cartesian3());
      var north = new Cesium.Cartesian3(0, 1, 0);
      var arc_north = Cesium.Cartesian3.dot(north, newC); // east用于判断与正北是否大于180度

      var east = new Cesium.Cartesian3(1, 0, 0);
      var arc_east = Cesium.Cartesian3.dot(east, aim);
      var radians_north = Math.acos(arc_north);
      var dg = Cesium.Math.toDegrees(radians_north);
      if (arc_east < 0) dg = 360 - dg;
      return dg;
    }
  }, {
    key: "formateLength",
    value: function formateLength(val, dw) {
      if (val == undefined) return;
      dw = dw || "m";
      var dwStr = '';

      if (dw == "km" || dw == "千米") {
        dwStr += (Number(val) / 1000).toFixed(2) + "km";
      } else if (dw == "m" || dw == "米") {
        dwStr += Number(val).toFixed(2) + "m";
      }

      return dwStr;
    }
  }, {
    key: "formateArea",
    value: function formateArea(val, dw) {
      if (val == undefined) return;
      var dwStr = '';
      dw = dw || "m";

      if (dw == "km" || dw == "平方千米") {
        dwStr += (Number(val) / 1000000).toFixed(2) + "km²";
      } else if (dw == "m" || dw == "米" || dw == "平方米") {
        dwStr += Number(val).toFixed(2) + "m²";
      }

      return dwStr;
    } //兼容模型和地形上坐标拾取

  }, {
    key: "getCatesian3FromPX",
    value: function getCatesian3FromPX(px, viewer) {
      if (!px) return undefined;
      px.x = px.x / this.scale[0];
      px.y = px.y / this.scale[1];
      var picks = viewer.scene.drillPick(px);
      viewer.scene.render();
      var cartesian;
      var isOn3dtiles = false;

      for (var i = 0; i < picks.length; i++) {
        if (picks[i] && picks[i].primitive && picks[i].primitive instanceof Cesium.Cesium3DTileset) {
          //模型上拾取
          isOn3dtiles = true;
          break;
        }
      }

      if (isOn3dtiles) {
        cartesian = viewer.scene.pickPosition(px);
      } else {
        var ray = viewer.camera.getPickRay(px);
        if (!ray) return null;
        cartesian = viewer.scene.globe.pick(ray, viewer.scene);
        //sin add 20240813
        if (!cartesian) {
          cartesian = this.viewer.camera.pickEllipsoid(
              px,
              this.viewer.scene.globe.ellipsoid
          );
        }
      }

      return cartesian;
    } // 获取长度

  }, {
    key: "getGroundLength",
    value: function getGroundLength(positions, callback) {
      var that = this;
      var ellipsoid = this.viewer.scene.globe.ellipsoid;
      var len = this.getLength(positions[0], positions[1]);

      if (!this.viewer.terrainProvider.availability) {
        console.log("缺少地形数据，或地形加载失败！");
        if (callback) callback(len);
        return;
      }

      var surfacePositions = Cesium.PolylinePipeline.generateArc({
        positions: positions,
        granularity: 0.00001
      });
      if (!surfacePositions) return;
      var cartographicArray = [];
      var tempHeight = Cesium.Cartographic.fromCartesian(positions[0]).height;

      for (var i = 0; i < surfacePositions.length; i += 3) {
        var cartesian = Cesium.Cartesian3.unpack(surfacePositions, i);
        cartographicArray.push(ellipsoid.cartesianToCartographic(cartesian));
      }

      Cesium.sampleTerrainMostDetailed(that.viewer.terrainProvider, cartographicArray).then(function (updateLnglats) {
        var allLength = 0;
        var offset = 10.0;

        for (var i = 0; i < updateLnglats.length; i++) {
          var item = updateLnglats[i];

          if (!item.height) {
            //当未获取到当前坐标下的地形高度时 手动设置为初始点的高度
            item.height = tempHeight;
          } else {
            item.height += offset;
          }
        }

        var raisedPositions = ellipsoid.cartographicArrayToCartesianArray(updateLnglats); //转为世界坐标数组

        for (var z = 0; z < raisedPositions.length - 1; z++) {
          allLength += Cesium.Cartesian3.distance(raisedPositions[z], raisedPositions[z + 1]);
        }

        if (allLength) callback(allLength);
      });
    } // 坡度量算

  }, {
    key: "getSlope",
    value: function getSlope(position, callback) {
      if (!position) return; // 求出该点周围两点的坐标 构建平面

      var ctg = Cesium.Cartographic.fromCartesian(position);
      var random = 1 / 100000;
      var lat = Cesium.Math.toDegrees(ctg.latitude);
      var lng = Cesium.Math.toDegrees(ctg.longitude);
      var height = ctg.height;
      var newCtg1 = Cesium.Cartographic.fromDegrees(lng, lat + random);
      var newCtg2 = Cesium.Cartographic.fromDegrees(lng + random, lat);
      var that = this;
      Cesium.sampleTerrainMostDetailed(this.viewer.terrainProvider, [newCtg1, newCtg2]).then(function (updateLnglats) {
        for (var i = 0; i < updateLnglats.length; i++) {
          var item = updateLnglats[i];
          item.height = item.height ? item.height : height;
        }

        var raisedPositions = that.viewer.scene.globe.ellipsoid.cartographicArrayToCartesianArray(updateLnglats); //转为世界坐标数组

        var newPosition1 = raisedPositions[0];
        var newPosition2 = raisedPositions[1];
        var mtx = Cesium.Transforms.eastNorthUpToFixedFrame(position);
        var mtx_inverse = Cesium.Matrix4.inverse(mtx, new Cesium.Matrix4());
        position = Cesium.Matrix4.multiplyByPoint(mtx_inverse, position, new Cesium.Cartesian3());
        newPosition1 = Cesium.Matrix4.multiplyByPoint(mtx_inverse, newPosition1, new Cesium.Cartesian3());
        newPosition2 = Cesium.Matrix4.multiplyByPoint(mtx_inverse, newPosition2, new Cesium.Cartesian3());
        var v1 = Cesium.Cartesian3.subtract(newPosition1, position, new Cesium.Cartesian3());
        var v2 = Cesium.Cartesian3.subtract(newPosition2, position, new Cesium.Cartesian3());
        var cross = Cesium.Cartesian3.cross(v1, v2, new Cesium.Cartesian3());
        cross = Cesium.Cartesian3.normalize(cross, new Cesium.Cartesian3());
        var z = new Cesium.Cartesian3(0, 0, 1);
        var arc = Cesium.Cartesian3.dot(cross, z);
        var radians_north = Math.acos(arc);
        var dg = Cesium.Math.toDegrees(radians_north);
        dg = dg > 90 ? 180 - dg : dg;
        if (callback) callback(dg);
      });
    }
  }, {
    key: "getLength",
    value: function getLength(c1, c2) {
      if (!c1 || !c2) return 0;
      return Cesium.Cartesian3.distance(c1, c2) || 0;
    } //调用第三方插件计算面积 turf

  }, {
    key: "getAreaAndCenter",
    value: function getAreaAndCenter(positions) {
      if (!positions || positions.length < 1) return;
      var cartographics = [];
      var turfPoints = [];

      for (var i = 0; i < positions.length; i++) {
        var cartesian3 = positions[i];
        var cartographic = Cesium.Cartographic.fromCartesian(cartesian3);
        cartographics.push([Cesium.Math.toDegrees(cartographic.longitude), Cesium.Math.toDegrees(cartographic.latitude)]);
        turfPoints.push(point([Cesium.Math.toDegrees(cartographic.longitude), Cesium.Math.toDegrees(cartographic.latitude)]));
      }

      if (!cartographics.length) return;
      cartographics = cartographics.concat([cartographics[0]]);
      var polygon$1 = polygon([cartographics]);
      var area$1 = area(polygon$1); //获取当前范围的中心点

      var features = featureCollection(turfPoints);
      var turfCenter = center(features);
      var center$1 = turfCenter.geometry.coordinates;
      return {
        area: area$1,
        center: Cesium.Cartesian3.fromDegrees(center$1[0], center$1[1])
      };
    } // 构建控制点

  }, {
    key: "createPoint",
    value: function createPoint(position) {
      if (!position) return;
      this.pointStyle.color = this.pointStyle.color || Cesium.Color.AQUA;
      this.pointStyle.outlineColor = this.pointStyle.color || Cesium.Color.WHITE;
      var color = this.pointStyle.color instanceof Cesium.Color ? this.pointStyle.color : Cesium.Color.fromCssColorString(this.pointStyle.color);
      color = color.withAlpha(this.pointStyle.colorAlpha || 0.8);
      var outlineColor = this.pointStyle.outlineColor instanceof Cesium.Color ? this.pointStyle.outlineColor : Cesium.Color.fromCssColorString(this.pointStyle.outlineColor);
      outlineColor = outlineColor.withAlpha(this.pointStyle.outlineColorAlpha || 0.8);
      return this.viewer.entities.add({
        position: position,
        point: {
          pixelSize: this.pointStyle.pixelSize || 6,
          color: color,
          outlineWidth: this.pointStyle.outlineWidth || 1,
          outlineColor: outlineColor,
          disableDepthTestDistance: Number.POSITIVE_INFINITY
        },
        show: false
      });
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {}
    /**
     * 结束编辑
     */

  }, {
    key: "endEdit",
    value: function endEdit() {}
    /**
     * 
     * 开始编辑
     * @param {Function} callback 编辑成功后回调函数
     */

  }, {
    key: "startEdit",
    value: function startEdit() {}
    /**
     * 开始绘制
     * @param {Function} callback 绘制成功后回调函数
     */

  }, {
    key: "start",
    value: function start() {}
  }]);

  return BaseMeasure;
}();

/**
 * 贴地距离测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureGroundDistance 
 */

var MeasureGroundDistance = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureGroundDistance, _BaseMeasure);

  var _super = _createSuper(MeasureGroundDistance);

  function MeasureGroundDistance(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureGroundDistance);

    _this = _super.call(this, viewer, opt);
    _this.unitType = "length";
    _this.unit = opt.unit || "米";
    _this.type = "groundDistance";
    if (!opt) opt = {};
    _this.style = opt.style || {};
    _this.viewer = viewer; //线

    _this.polyline = null; //线坐标

    _this.positions = []; //标签数组

    _this.labels = [];
    _this.nowLabel = null; // 编辑时  当前点的label

    _this.nextlabel = null; // 编辑时  下一个点的label

    _this.lastPosition = null; // 编辑时   上一个点的坐标

    _this.nextPosition = null; // 编辑时   下一个点的坐标

    _this.modifyPoint = null;
    _this.lastCartesian = null;
    _this.allDistance = 0;
    _this.prompt;
    _this.movePush = false;
    _this.floatDistance = -1;
    return _this;
  } //开始测量


  _createClass(MeasureGroundDistance, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      var that = this;
      this.state = "startCreate";
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        if (!that.floatLable) {
          that.floatLable = that.createLabel(cartesian, "");
          that.floatLable.wz = 0;
          that.floatLable.show = false;
        }

        var label = that.createLabel(cartesian, "");
        label.wz = that.positions.length;
        that.labels.push(label);
        var point = that.createPoint(cartesian.clone());
        point.wz = that.positions.length;
        that.controlPoints.push(point);

        if (that.positions.length == 0) {
          label.label.text = "起点";
        } else {
          that.lastDistance = that.floatDistance;
          that.allDistance += that.floatDistance;
          var text = that.formateLength(that.floatDistance);
          label.label.text = text;
          label.distance = that.floatDistance;
        }

        that.positions.push(cartesian.clone());
        that.lastCartesian = cartesian.clone();
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";

        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始测量");
          return;
        }

        that.prompt.update(evt.endPosition, "双击结束，右键取消上一步");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (!that.movePush) {
          that.positions.push(cartesian);
          that.movePush = true;
        } else {
          that.positions[that.positions.length - 1] = cartesian.clone();
        }

        if (!Cesium.defined(that.polyline)) {
          that.polyline = that.createLine(that.positions, true);
          that.polyline.objId = that.objId;
        }

        if (!that.lastCartesian) return;
        that.getGroundLength([cartesian, that.lastCartesian], function (distance) {
          that.floatLable.show = true;
          that.floatLable.label.text = that.formateLength(distance, that.unit);
          that.floatLable.position.setValue(cartesian);
          that.floatLable.distance = distance;
          if (distance) that.floatDistance = distance;
          /* if (that.fun) that.fun(distance); */
        });
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        if (!that.polyline) return;
        if (that.positions.length <= 2) return; // 默认最后一个不给删除

        that.positions.splice(that.positions.length - 2, 1);
        that.viewer.entities.remove(that.labels.pop());
        that.viewer.entities.remove(that.controlPoints.pop()); // 移除最后一个

        that.allDistance = that.allDistance - that.lastDistance;

        if (that.positions.length == 1) {
          if (that.polyline) {
            that.viewer.entities.remove(that.polyline);
            that.polyline = null;
          }

          that.prompt.update(evt.endPosition, "单击开始测量");
          that.floatLable.show = false;
          that.positions = [];
        }

        if (!that.movePush) {
          that.lastCartesian = that.positions[that.positions.length - 1];
        } else {
          that.lastCartesian = that.positions[that.positions.length - 2];
        }

        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;
        that.getGroundLength([cartesian, that.positions[that.positions.length - 2]], function (distance) {
          that.floatLable.show = true;
          that.floatLable.label.text = that.formateLength(distance, that.unit);
          that.floatLable.distance = distance;
          that.floatLable.position.setValue(cartesian);
        });
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
      this.handler.setInputAction(function (evt) {
        //双击结束绘制
        // 移除双击事件多生成的最后一个 
        that.positions.pop();
        that.viewer.entities.remove(that.labels.pop());
        that.viewer.entities.remove(that.controlPoints.pop());
        that.movePush = false;
        that.endCreate();
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      if (!that.polyline) return;
      that.floatLable.show = false;
      that.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      that.viewer.trackedEntity = undefined;
      var allDistance = that.formateLength(that.allDistance/2, that.unit);
      that.labels[that.labels.length - 1].label.text = "总长：" + allDistance;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    } // 开始编辑

  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!((this.state == "endCreate" || this.state == "endEdit") && this.polyline)) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);

        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
          var wz = that.modifyPoint.wz; // 重新计算左右距离

          var nextIndex = wz + 1;
          var lastIndex = wz - 1;
          that.nowLabel = that.labels[wz];

          if (lastIndex >= 0) {
            that.lastPosition = that.positions[lastIndex];
          }

          if (nextIndex <= that.positions.length - 1) {
            that.nextPosition = that.positions[nextIndex];
            that.nextlabel = that.labels[nextIndex];
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian);
        var wz = that.modifyPoint.wz;
        that.positions[wz] = cartesian.clone();
        that.state = "editing";
        that.nowLabel.position.setValue(cartesian.clone());
        var changeDis1 = 0;
        var changeDis2 = 0;

        if (that.nowLabel && that.lastPosition) {
          that.getGroundLength([cartesian.clone(), that.lastPosition.clone()], function (distance) {
            that.nowLabel.label.text = that.formateLength(distance, that.unit);
            changeDis1 = distance - that.nowLabel.distance;
            that.nowLabel.distance = distance; // 计算总长

            // that.allDistance = that.allDistance + changeDis1 + changeDis2;
            // var allDistance = that.formateLength(that.allDistance, that.unit);
            // that.labels[that.labels.length - 1].label.text = "总长：" + allDistance;
          });
        }

        if (that.nextPosition && that.nextlabel) {
          that.getGroundLength([cartesian.clone(), that.nextPosition.clone()], function (distance) {
            that.nextlabel.label.text = that.formateLength(distance, that.unit);
            changeDis2 = distance - that.nextlabel.distance;
            that.nextlabel.distance = distance; // 计算总长

            that.allDistance = that.allDistance + changeDis1 + changeDis2;
            var allDistance = that.formateLength(that.allDistance, that.unit);
            that.labels[that.labels.length - 1].label.text = "总长：" + allDistance;
          });
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.lastPosition = null;
        that.nextPosition = null;
        that.forbidDrawWorld(false);
        if (callback) callback();
        that.state = "endEdit";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
    /**
     * 结束编辑
     */

  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //sin 2024.7.17 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      for (var i = 0; i < this.labels.length; i++) {
        this.viewer.entities.remove(this.labels[i]);
      }

      this.labels = [];

      for (var ind = 0; ind < this.controlPoints.length; ind++) {
        this.viewer.entities.remove(this.controlPoints[ind]);
      }

      this.controlPoints = [];
      this.modifyPoint = null;

      if (this.floatLable) {
        this.viewer.entities.remove(this.floatLable);
        this.floatLable = null;
      }

      this.floatLable = null;

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      this.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      this.viewer.trackedEntity = undefined;
      this.state = "no";
    } // 设置单位

  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      for (var i = 1; i < this.labels.length; i++) {
        var labelEnt = this.labels[i];
        var distance = labelEnt.distance;
        var label = labelEnt.label;
        if (!label) continue;

        if (i == this.labels.length - 1) {
          label.text = "总长：" + this.formateLength(distance, unit);
        } else {
          label.text = this.formateLength(distance, unit);
        }
      }

      this.unit = unit;
    }
  }]);

  return MeasureGroundDistance;
}(BaseMeasure);

/**
 * 空间距离测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureSpaceDistance 
 */

var MeasureSpaceDistance = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureSpaceDistance, _BaseMeasure);

  var _super = _createSuper(MeasureSpaceDistance);

  function MeasureSpaceDistance(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureSpaceDistance);

    _this = _super.call(this, viewer, opt);
    _this.unitType = "length";
    _this.type = "spaceDistance";
    _this.unit = opt.unit || "米";
    /**
     * @property {Number} allDistance 总长度
     */

    _this.allDistance = 0;
    _this.labels = [];
    _this.positions = []; //线

    _this.polyline = null;
    _this.nowLabel = null; // 编辑时  当前点的label

    _this.nextlabel = null; // 编辑时  下一个点的label

    _this.lastPosition = null; // 编辑时   上一个点的坐标

    _this.nextPosition = null; // 编辑时   下一个点的坐标

    return _this;
  } //开始测量


  _createClass(MeasureSpaceDistance, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      var that = this;
      this.state = "startCreate";
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        var label;

        if (that.positions.length == 0) {
          label = that.createLabel(cartesian, "起点");
          that.floatLable = that.createLabel(cartesian, "");
          that.floatLable.wz = 0;
          that.floatLable.show = false;
        } else {
          var distance = that.getLength(cartesian, that.lastCartesian);
          that.lastDistance = distance;
          that.allDistance += distance;
          var text = that.formateLength(distance, that.unit);
          label = that.createLabel(cartesian, text);
          label.wz = that.positions.length; // 和坐标点关联

          label.distance = distance;
        }

        that.labels.push(label);
        var point = that.createPoint(cartesian.clone());
        point.wz = that.positions.length; // 和坐标点关联

        that.controlPoints.push(point);
        that.positions.push(cartesian);
        that.lastCartesian = cartesian.clone();
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.state = "creating";

        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始测量");
          return;
        } else {
          that.prompt.update(evt.endPosition, "双击结束，右键取消上一步");
          that.floatLable.show = true;

          if (!that.movePush) {
            that.positions.push(cartesian);
            that.movePush = true;
          } else {
            that.positions[that.positions.length - 1] = cartesian;
          }

          if (!Cesium.defined(that.polyline)) {
            that.polyline = that.createLine(that.positions, false);
            that.polyline.objId = that.objId;
          }

          if (!that.lastCartesian) return;
          var distance = that.getLength(cartesian, that.lastCartesian);
          that.floatLable.show = true;
          that.floatLable.label.text = that.formateLength(distance, that.unit);
          that.floatLable.distance = distance;
          that.floatLable.position.setValue(cartesian);
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        if (!that.polyline) return;
        if (that.positions.length <= 2) return; // 默认第一个不给删除

        that.positions.splice(that.positions.length - 2, 1);
        that.viewer.entities.remove(that.labels.pop());
        that.viewer.entities.remove(that.controlPoints.pop()); // 移除最后一个

        that.allDistance = that.allDistance - that.lastDistance;

        if (that.positions.length == 1) {
          if (that.polyline) {
            that.viewer.entities.remove(that.polyline);
            that.polyline = null;
          }

          that.prompt.update(evt.endPosition, "单击开始测量");
          that.movePush = false;
          that.floatLable.show = false;
          that.positions = [];
        }

        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;
        var distance = that.getLength(cartesian, that.positions[that.positions.length - 2]);
        that.floatLable.show = true;
        that.floatLable.label.text = that.formateLength(distance, that.unit);
        that.floatLable.distance = distance;
        that.floatLable.position.setValue(cartesian);

        if (!that.movePush) {
          that.lastCartesian = that.positions[that.positions.length - 1];
        } else {
          that.lastCartesian = that.positions[that.positions.length - 2];
        }
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
      this.handler.setInputAction(function (evt) {
        //双击结束绘制
        if (!that.polyline) return;
        that.positions.pop();
        that.viewer.entities.remove(that.labels.pop());
        that.viewer.entities.remove(that.controlPoints.pop()); // 移除最后一个

        that.movePush = false;
        that.endCreate();
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      if (!that.polyline) return;
      that.floatLable.show = false;
      that.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      that.viewer.trackedEntity = undefined;
      var allDistance = that.formateLength(that.allDistance, that.unit);
      that.labels[that.labels.length - 1].label.text = "总长：" + allDistance;

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    } // 开始编辑

  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!(this.state == "endCreate" || this.state == "endEdit")) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
          var wz = that.modifyPoint.wz; // 重新计算左右距离

          var nextIndex = wz + 1;
          var lastIndex = wz - 1;
          that.nowLabel = that.labels[wz];

          if (lastIndex >= 0) {
            that.lastPosition = that.positions[lastIndex];
          }

          if (nextIndex <= that.positions.length - 1) {
            that.nextPosition = that.positions[nextIndex];
            that.nextlabel = that.labels[nextIndex];
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian);
        var wz = that.modifyPoint.wz;
        that.positions[wz] = cartesian.clone();
        that.state = "editing";
        that.nowLabel.position.setValue(cartesian.clone());
        var changeDis1 = 0;

        if (that.nowLabel && that.lastPosition) {
          var distance = that.getLength(cartesian.clone(), that.lastPosition.clone());
          that.nowLabel.label.text = that.formateLength(distance, that.unit);
          changeDis1 = distance - that.nowLabel.distance;
          that.nowLabel.distance = distance;
        }

        var changeDis2 = 0;

        if (that.nextPosition && that.nextlabel) {
          var _distance = that.getLength(cartesian.clone(), that.nextPosition.clone());

          that.nextlabel.label.text = that.formateLength(_distance, that.unit);
          changeDis2 = _distance - that.nextlabel.distance;
          that.nextlabel.distance = _distance;
        } // 计算总长


        that.allDistance = that.allDistance + changeDis1 + changeDis2;
        var allDistance = that.formateLength(that.allDistance, that.unit);
        that.labels[that.labels.length - 1].label.text = "总长：" + allDistance;
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.lastPosition = null;
        that.nextPosition = null;
        that.forbidDrawWorld(false);
        if (callback) callback();
        that.state = "endEdit";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //sin 2024.7.17 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      for (var i = 0; i < this.labels.length; i++) {
        this.viewer.entities.remove(this.labels[i]);
      }

      this.labels = [];

      for (var ind = 0; ind < this.controlPoints.length; ind++) {
        this.viewer.entities.remove(this.controlPoints[ind]);
      }

      this.controlPoints = [];
      this.modifyPoint = null;

      if (this.floatLable) {
        this.viewer.entities.remove(this.floatLable);
        this.floatLable = null;
      }

      this.floatLable = null;

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      this.movePush = false;
      this.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      this.viewer.trackedEntity = undefined;
      this.state = "no";
    } // 设置单位

  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      for (var i = 1; i < this.labels.length; i++) {
        var labelEnt = this.labels[i];
        var distance = labelEnt.distance;
        var label = labelEnt.label;
        if (!label) continue;

        if (i == this.labels.length - 1) {
          label.text = "总长：" + this.formateLength(distance, unit);
        } else {
          label.text = this.formateLength(distance, unit);
        }
      }

      this.unit = unit;
    }
  }]);

  return MeasureSpaceDistance;
}(BaseMeasure);

/**
 * 空间面积测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureSpaceArea 
 */

var MeasureSpaceArea = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureSpaceArea, _BaseMeasure);

  var _super = _createSuper(MeasureSpaceArea);

  function MeasureSpaceArea(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureSpaceArea);

    _this = _super.call(this, viewer, opt);
    if (!opt) opt = {};
    _this.unitType = "area";
    _this.unit = opt.unit || "平方米";
    _this.style = opt.style || {};
    _this.viewer = viewer;
    _this.polyline = null;
    _this.polygon = null; //面积标签

    _this.positions = [];
    _this.movePush = false;
    _this.prompt = undefined;
    return _this;
  } //开始测量


  _createClass(MeasureSpaceArea, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      var that = this;
      this.state = "startCreate";
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        var point = that.createPoint(cartesian.clone());
        point.wz = that.positions.length; // 和坐标点关联

        that.controlPoints.push(point);
        that.positions.push(cartesian);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";

        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          return;
        }

        that.prompt.update(evt.endPosition, "双击结束，右键取消上一步");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if(!cartesian) {
          if(!that.posOld){
            that.posOld = {x: 5216762.434938885, y: 2603807.953920882, z: 2574439.3261103444}
          }
          cartesian = that.posOld
        }else {
          that.posOld = cartesian
        }
        if (that.positions.length >= 1) {
          if (!that.movePush) {
            that.positions.push(cartesian);
            that.movePush = true;
          } else {
            that.positions[that.positions.length - 1] = cartesian;
          }

          if (that.positions.length == 2) {
            if (!Cesium.defined(that.polyline)) {
              that.polyline = that.createPolyline();
            }
          }

          if (that.positions.length == 3) {
            if (!Cesium.defined(that.polygon)) {
              that.polygon = that.createPolygon();
              that.polygon.isFilter = true;
              that.polygon.objId = that.objId;
              /* if (that.polyline) that.polyline.show = false; */
            }

            if (!that.floatLabel) {
              that.floatLabel = that.createLabel(cartesian, "");
              //wzt处理二位面积大小文字出不来问题
              // that.floatLabel.label.heightReference = 1;
            }
          }

          if (that.polygon) {
            var areaCenter = that.getAreaAndCenter(that.positions);
            var area = areaCenter.area;
            var center = areaCenter.center;
            var text = that.formateArea(area, that.unit);
            that.floatLabel.label.text = "面积：" + text;
            that.floatLabel.area = area;
            if (center) that.floatLabel.position.setValue(center);
            that.floatLabel.show = true;
          }
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        if (!that.polyline && !that.polygon) return;
        that.positions.splice(that.positions.length - 2, 1);
        that.viewer.entities.remove(that.controlPoints.pop());

        if (that.positions.length == 2) {
          if (that.polygon) {
            that.viewer.entities.remove(that.polygon);
            that.polygon = null;
            if (that.polyline) that.polyline.show = true;
          }

          that.floatLabel.show = false;
        }

        if (that.positions.length == 1) {
          if (that.polyline) {
            that.viewer.entities.remove(that.polyline);
            that.polyline = null;
          }

          that.prompt.update(evt.endPosition, "单击开始测量");
          that.positions = [];
          that.movePush = false;
        }

        if (that.positions.length > 2) {
          var areaCenter = that.getAreaAndCenter(that.positions);
          var area = areaCenter.area;
          var center = areaCenter.center;
          var text = that.formateArea(area, that.unit);
          that.floatLabel.label.text = "面积：" + text;
          if (center) that.floatLabel.position.setValue(center);
          that.floatLabel.area = area;
          that.floatLabel.show = true;
        }
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
      this.handler.setInputAction(function (evt) {
        //双击结束绘制
        //sin del 20240710
/*        if (!that.polygon) {
          return;
        }*/

        that.positions.pop();
        that.viewer.entities.remove(that.controlPoints.pop()); // 移除最后一个

        that.movePush = false;
        that.endCreate();
        if (callback) callback(that.polyline);
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      that.viewer.trackedEntity = undefined;
      var areaCenter = that.getAreaAndCenter(that.positions);
      var area = areaCenter.area;
      var center = areaCenter.center;
      var text = that.formateArea(area, that.unit);
      that.floatLabel.label.text = "面积：" + text;
      that.floatLabel.area = area;
      if (center) that.floatLabel.position.setValue(center);

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!((this.state == "endCreate" || this.state == "endEdit") && this.polygon)) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);
        
        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian);
        var wz = that.modifyPoint.wz;
        that.positions[wz] = cartesian.clone();
        var areaCenter = that.getAreaAndCenter(that.positions);
        var area = areaCenter.area;
        var center = areaCenter.center;
        var text = that.formateArea(area, that.unit);
        that.floatLabel.label.text = "面积：" + text;
        that.floatLabel.area = area;
        if (center) that.floatLabel.position.setValue(center);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.forbidDrawWorld(false);
        if (callback) callback();
        that.state = "endEdit";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //wzt 2024.7.10 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      this.state = "no";

      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      if (this.polygon) {
        this.viewer.entities.remove(this.polygon);
        this.polygon = null;
      }

      if (this.floatLabel) {
        this.viewer.entities.remove(this.floatLabel);
        this.floatLabel = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      for (var i = 0; i < this.controlPoints.length; i++) {
        var point = this.controlPoints[i];
        this.viewer.entities.remove(point);
      }

      this.floatLable = null;
    }
  }, {
    key: "createPolyline",
    value: function createPolyline() {
      var that = this;
      var polyline = this.viewer.entities.add({
        polyline: {
          positions: new Cesium.CallbackProperty(function () {
            var linePositions = that.positions.concat([that.positions[0]]);
            return linePositions;
          }, false),
          material: Cesium.Color.GOLD,
          width: 2,
          clampToGround: true
        }
      });
      return polyline;
    }
  }, {
    key: "createPolygon",
    value: function createPolygon() {
      var that = this;
      var polygon = this.viewer.entities.add({
        polygon: new Cesium.PolygonGraphics({
          hierarchy: new Cesium.CallbackProperty(function () {
            return new Cesium.PolygonHierarchy(that.positions);
          }, false),
          material: this.style.material || Cesium.Color.WHITE.withAlpha(0.6),
          fill: true
        })
      });
      return polygon;
    }
  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      this.unit = unit;
      var text = this.formateArea(this.floatLabel.area, unit);
      this.floatLabel.label.text = "面积：" + text;
    }
  }]);

  return MeasureSpaceArea;
}(BaseMeasure);

/**
 * 高度测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureHeight 
 */

var MeasureHeight = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureHeight, _BaseMeasure);

  var _super = _createSuper(MeasureHeight);

  function MeasureHeight(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureHeight);

    _this = _super.call(this, viewer, opt);
    if (!opt) opt = {};
    _this.unitType = "length";
    _this.unit = opt.unit || "米";
    _this.style = opt.style || {};
    _this.viewer = viewer;
    _this.polyline = null;
    _this.floatLabel = null;
    _this.positions = [];
    return _this;
  } //开始测量


  _createClass(MeasureHeight, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.state = "startCreate";
      var that = this;
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (that.positions.length == 2) {
          that.positions[1] = cartesian.clone();
          var point = that.createPoint(cartesian.clone());
          point.wz = 1;
          that.controlPoints.push(point);
          that.endCreate();
          if (callback) callback();
        } else {
          that.polyline = that.createLine(that.positions, false);
          that.polyline.objId = that.objId;
          if (!that.floatLabel) that.floatLabel = that.createLabel(cartesian.clone(), "");
          that.positions.push(cartesian.clone());

          var _point = that.createPoint(cartesian.clone());

          _point.wz = 0;
          that.controlPoints.push(_point);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.state = "creating";

        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始测量");
          return;
        }

        that.prompt.update(evt.endPosition, "单击结束");
        if (!cartesian) return;

        if (that.positions.length < 2) {
          that.positions.push(cartesian.clone());
        } else {
          that.positions[1] = cartesian.clone();
        }

        var heightAndCenter = that.getHeightAndCenter(that.positions[0], that.positions[1]);
        var text = that.formateLength(heightAndCenter.height, that.unit);
        that.floatLabel.label.text = "高度差：" + text;
        that.floatLabel.length = heightAndCenter.height;
        if (heightAndCenter.center) that.floatLabel.position.setValue(heightAndCenter.center);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      } else {
        this.endCreate();
      }
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!((this.state == "endCreate" || this.state == "endEdit") && this.polyline)) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);

        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian.clone());
        that.positions[that.modifyPoint.wz] = cartesian.clone();
        var heightAndCenter = that.getHeightAndCenter(that.positions[0], that.positions[1]);
        var text = that.formateLength(heightAndCenter.height, that.unit);
        that.floatLabel.label.text = "高度差：" + text;
        that.floatLabel.length = heightAndCenter.height;
        if (heightAndCenter.center) that.floatLabel.position.setValue(heightAndCenter.center);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.lastPosition = null;
        that.nextPosition = null;
        that.forbidDrawWorld(false);
        if (callback) callback();
        that.state = "endEdit";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //sin 2024.7.17 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      if (this.floatLabel) {
        this.viewer.entities.remove(this.floatLabel);
        this.floatLabel = null;
      }

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      this.state = "no";
    }
  }, {
    key: "getHeightAndCenter",
    value: function getHeightAndCenter(p1, p2) {
      if (!p1 || !p2) return;
      var cartographic1 = Cesium.Cartographic.fromCartesian(p1);
      var cartographic2 = Cesium.Cartographic.fromCartesian(p2);
      var height = Math.abs(cartographic1.height - cartographic2.height);
      return {
        height: height,
        center: Cesium.Cartesian3.midpoint(p1, p2, new Cesium.Cartesian3())
      };
    }
  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      var text = this.formateLength(this.floatLabel.length, unit);
      this.floatLabel.label.text = "高度差：" + text;
      this.unit = unit;
    }
  }]);

  return MeasureHeight;
}(BaseMeasure);

/**
 * 三角测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureTriangle 
 */

var MeasureTriangle = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureTriangle, _BaseMeasure);

  var _super = _createSuper(MeasureTriangle);

  function MeasureTriangle(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureTriangle);

    _this = _super.call(this, viewer, opt);
    if (!opt) opt = {};
    _this.unitType = "length";
    _this.style = opt.style || {}; //线

    _this.heightfloatLabel = null;
    _this.spaceDistancefloatLabel = null;
    _this.horizonDistancefloatLabel = null;
    _this.heightLine = null;
    _this.spaceLine = null;
    _this.horizonLine = null;
    _this.firstPosition = null;
    _this.endPosition = null;
    _this.midPosition = undefined;
    _this.lowPosition = undefined;
    _this.highPosition = undefined;
    return _this;
  } //开始测量


  _createClass(MeasureTriangle, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      var that = this;
      this.state = 1;
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (!that.firstPosition) {
          that.firstPosition = cartesian.clone();
          that.heightfloatLabel = that.createLabel(cartesian, "");
          that.spaceDistancefloatLabel = that.createLabel(cartesian, "");
          that.horizonDistancefloatLabel = that.createLabel(cartesian, "");
          var point = that.createPoint(cartesian.clone());
          point.wz = 0;
          that.controlPoints.push(point);
        } else {
          that.endPosition = cartesian;
          that.computerPosition(that.firstPosition, that.endPosition);

          var _point = that.createPoint(cartesian.clone());

          _point.wz = 1;
          that.controlPoints.push(_point);

          if (that.handler) {
            that.handler.destroy();
            that.handler = null;
          }

          if (that.prompt) {
            that.prompt.destroy();
            that.prompt = null;
          }

          that.state = "endCreate";
          if (callback) callback();
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";

        if (!that.firstPosition) {
          that.prompt.update(evt.endPosition, "单击开始测量");
          return;
        }

        that.prompt.update(evt.endPosition, "单击结束");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.endPosition = cartesian;
        that.computerPosition(that.firstPosition, that.endPosition);

        if (that.firstPosition && that.endPosition && !that.spaceLine) {
          that.spaceLine = that.viewer.entities.add({
            polyline: {
              positions: new Cesium.CallbackProperty(function () {
                return [that.firstPosition, that.endPosition];
              }, false),
              show: true,
              material: new Cesium.PolylineOutlineMaterialProperty({
                color: Cesium.Color.GOLD,
                outlineWidth: 2,
                outlineColor: Cesium.Color.BLACK
              }),
              width: 3
            }
          });
          that.spaceLine.objId = that.objId;
          that.heightLine = that.viewer.entities.add({
            polyline: {
              positions: new Cesium.CallbackProperty(function () {
                return [that.lowPosition, that.midPosition];
              }, false),
              show: true,
              material: new Cesium.PolylineOutlineMaterialProperty({
                color: Cesium.Color.GOLD,
                outlineWidth: 2,
                outlineColor: Cesium.Color.BLACK
              }),
              width: 3
            }
          });
          that.heightLine.objId = that.objId;
          that.horizonLine = that.viewer.entities.add({
            polyline: {
              positions: new Cesium.CallbackProperty(function () {
                return [that.highPosition, that.midPosition];
              }, false),
              show: true,
              material: new Cesium.PolylineOutlineMaterialProperty({
                color: Cesium.Color.GOLD,
                outlineWidth: 2,
                outlineColor: Cesium.Color.BLACK
              }),
              width: 3
            }
          });
          that.horizonLine.objId = that.objId;
        }

        if (that.spaceLine) that.createLabels();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    } //计算正上方的点

  }, {
    key: "computerPosition",
    value: function computerPosition(p1, p2) {
      var cartographic1 = Cesium.Cartographic.fromCartesian(p1.clone());
      var cartographic2 = Cesium.Cartographic.fromCartesian(p2.clone());

      if (cartographic1.height > cartographic2.height) {
        this.highPosition = p1.clone();
        this.lowPosition = p2.clone();
        this.midPosition = Cesium.Cartesian3.fromRadians(cartographic2.longitude, cartographic2.latitude, cartographic1.height);
      } else {
        this.lowPosition = p1.clone();
        this.highPosition = p2.clone();
        this.midPosition = Cesium.Cartesian3.fromRadians(cartographic1.longitude, cartographic1.latitude, cartographic2.height);
      }
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!(this.state == "endCreate" || this.state == "endEdit")) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);

        if (Cesium.defined(pick) && pick.id) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian.clone());

        if (that.modifyPoint.wz == 0) {
          that.firstPosition = cartesian.clone();
        } else {
          that.endPosition = cartesian.clone();
        }

        that.computerPosition(that.firstPosition, that.endPosition);
        that.createLabels();
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.forbidDrawWorld(false);
        that.state = "endEdit";
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //sin 2024.7.17 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    }
  }, {
    key: "createLabels",
    value: function createLabels() {
      var that = this; //高度差

      var height = Math.abs(Cesium.Cartographic.fromCartesian(that.highPosition).height - Cesium.Cartographic.fromCartesian(that.lowPosition).height);
      var height_mid = Cesium.Cartesian3.midpoint(that.lowPosition, that.midPosition, new Cesium.Cartesian3());
      that.heightfloatLabel.show = true;
      that.heightfloatLabel.position.setValue(height_mid);
      var text1 = that.formateLength(height, that.unit);
      that.heightfloatLabel.label.text = "高度差：" + text1;
      that.heightfloatLabel.length = height; //水平距离

      var horizonDistance = Cesium.Cartesian3.distance(that.highPosition, that.midPosition);
      var horizon_mid = Cesium.Cartesian3.midpoint(that.highPosition, that.midPosition, new Cesium.Cartesian3());
      that.horizonDistancefloatLabel.show = true;
      that.horizonDistancefloatLabel.position.setValue(horizon_mid);
      var text2 = that.formateLength(horizonDistance, that.unit);
      that.horizonDistancefloatLabel.label.text = "水平距离：" + text2;
      that.horizonDistancefloatLabel.length = horizonDistance; //空间距离

      var spaceDistance = Cesium.Cartesian3.distance(that.endPosition, that.firstPosition);
      var space_mid = Cesium.Cartesian3.midpoint(that.endPosition, that.firstPosition, new Cesium.Cartesian3());
      that.spaceDistancefloatLabel.show = true;
      that.spaceDistancefloatLabel.position.setValue(space_mid);
      var text3 = that.formateLength(spaceDistance, that.unit);
      that.spaceDistancefloatLabel.label.text = "空间距离：" + text3;
      that.spaceDistancefloatLabel.length = spaceDistance;
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      this.state = "no";

      if (this.heightLine) {
        this.viewer.entities.remove(this.heightLine);
        this.heightLine = null;
      }

      if (this.spaceLine) {
        this.viewer.entities.remove(this.spaceLine);
        this.spaceLine = null;
      }

      if (this.horizonLine) {
        this.viewer.entities.remove(this.horizonLine);
        this.horizonLine = null;
      }

      if (this.heightfloatLabel) {
        this.viewer.entities.remove(this.heightfloatLabel);
        this.heightfloatLabel = null;
      }

      this.heightfloatLabel = null;

      if (this.spaceDistancefloatLabel) {
        this.viewer.entities.remove(this.spaceDistancefloatLabel);
        this.spaceDistancefloatLabel = null;
      }

      this.spaceDistancefloatLabel = null;

      if (this.horizonDistancefloatLabel) {
        this.viewer.entities.remove(this.horizonDistancefloatLabel);
        this.horizonDistancefloatLabel = null;
      }

      this.horizonDistancefloatLabel = null;

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }
    }
  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      if (this.heightfloatLabel) {
        var text1 = this.formateLength(this.heightfloatLabel.length, unit);
        this.heightfloatLabel.label.text = "高度差：" + text1;
      }

      if (this.horizonDistancefloatLabel) {
        var text2 = this.formateLength(this.horizonDistancefloatLabel.length, unit);
        this.horizonDistancefloatLabel.label.text = "水平距离：" + text2;
      }

      if (this.spaceDistancefloatLabel) {
        var text3 = this.formateLength(this.spaceDistancefloatLabel.length, unit);
        this.spaceDistancefloatLabel.label.text = "空间距离：" + text3;
      }

      this.unit = unit;
    }
  }]);

  return MeasureTriangle;
}(BaseMeasure);

/**
 * 坐标测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureLnglat 
 */

var MeasureLnglat = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureLnglat, _BaseMeasure);

  var _super = _createSuper(MeasureLnglat);

  function MeasureLnglat(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureLnglat);

    _this = _super.call(this, viewer, opt);
    if (!opt) opt = {};
    _this.style = opt.style || {};
    _this.point = null;
    _this.position = null;
    _this.state = 0;
    return _this;
  }

  _createClass(MeasureLnglat, [{
    key: "start",
    value: function start(callback) {
      this.state = "startCreate";
      var that = this;
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        if (that.handler) {
          that.handler.destroy();
          that.handler = null;
        }

        that.state = "endCreate";
        if (callback) callback();
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.position = cartesian.clone();

        if (!Cesium.defined(that.point)) {
          that.point = that.createPoint();
          that.point.objId = that.objId;
        }

        var lnglat = util$1.cartesianToLnglat(cartesian, that.viewer);
        that.point.label.text = "经度：" + lnglat[0].toFixed(6) + "\n纬度：" + lnglat[1].toFixed(6) + "\n高度：" + lnglat[2].toFixed(2) + " m";
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }
      this.endEdit()

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      } else {
        this.endCreate();
      }
    }
  }, {
    key: "destroy",
    value: function destroy() {
      this.state = "no";

      if (this.point) {
        this.viewer.entities.remove(this.point);
        this.point = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }
    }
  }, {
    key: "createPoint",
    value: function createPoint() {
      var _point;

      var that = this;
      var point = this.viewer.entities.add({
        position: new Cesium.CallbackProperty(function () {
          return that.position;
        }, false),
        point: (_point = {
          show: true,
          outlineColor: Cesium.Color.YELLOW
        }, _defineProperty(_point, "outlineColor", Cesium.Color.WHITE), _defineProperty(_point, "pixelSize", 6), _defineProperty(_point, "outlineWidth", 2), _defineProperty(_point, "disableDepthTestDistance", Number.MAX_VALUE), _point),
        label: {
          font: '18px Helvetica',
          fillColor: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cesium.Cartesian2(0, -60)
        }
      });
      return point;
    }
  }]);

  return MeasureLnglat;
}(BaseMeasure);

/**
 * 方位角测量类
 * @class 
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureAzimutht 
 */

var MeasureAzimutht = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureAzimutht, _BaseMeasure);

  var _super = _createSuper(MeasureAzimutht);

  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   */
  function MeasureAzimutht(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureAzimutht);

    _this = _super.call(this, viewer, opt);
    /**
     * @property {Object} style 绘制样式（polyline），具体配置见{@link style};
     */

    _this.style = opt.style || {};
    /**
     * @property {Cesium.Entity} polyline 线
     */

    _this.polyline = null;
    _this.floatLabel = null;
    /**
    * @property {Cesium.Cartesian3[]} positions 线坐标数组
    */

    _this.positions = [];
    _this.mtx = null;
    _this.azimutht = null;
    return _this;
  }
  /**
   * 开始绘制
   * @param {Function} callback 绘制成功后回调函数
  */


  _createClass(MeasureAzimutht, [{
    key: "start",
    value: function start(callback) {
      var that = this;
      this.state = "startCreate";
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (that.positions.length == 2) {
          that.positions.pop();
          var point = that.createPoint(cartesian.clone());
          point.wz = 1;
          that.controlPoints.push(point);
          that.state = "endCreate";
          that.endCreate();
          if (callback) callback(that.azimutht);
        }

        if (!that.polyline) {
          that.polyline = that.createLine(that.positions, true);
          that.polyline.objId = that.objId;
          that.polyline.polyline.width = 6;
          that.polyline.polyline.material = new Cesium.PolylineArrowMaterialProperty(Cesium.Color.YELLOW);
        }

        that.positions.push(cartesian);

        if (that.positions.length == 1) {
          that.mtx = Cesium.Transforms.eastNorthUpToFixedFrame(that.positions[0].clone());
          that.floatLabel = that.createLabel(cartesian, "");

          var _point = that.createPoint(cartesian.clone());

          _point.wz = 0;
          that.controlPoints.push(_point);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始测量");
          return;
        }

        that.prompt.update(evt.endPosition, "单击结束");
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (that.positions.length < 2) {
          that.positions.push(cartesian.clone());
        } else {
          that.positions[1] = cartesian.clone();
        }

        if (that.floatLabel) {
          that.azimutht = that.getAzimuthtAndCenter(that.mtx, that.positions);
          that.floatLabel.label.text = "方位角：" + that.azimutht.toFixed(2);
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
    /**
     * 结束创建
     */

  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.state = "endCreate";

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      } else {
        this.endCreate();
      }
    }
    /**
     * 
     * 开始编辑
     * @param {Function} callback 编辑成功后回调函数
     */

  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!((this.state == "endCreate" || this.state == "endEdit") && this.polyline)) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);

        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian.clone());

        if (that.modifyPoint.wz == 0) {
          that.floatLabel.position.setValue(cartesian.clone());
          that.mtx = Cesium.Transforms.eastNorthUpToFixedFrame(that.positions[0].clone());
        }

        that.positions[that.modifyPoint.wz] = cartesian.clone();
        that.azimutht = that.getAzimuthtAndCenter(that.mtx, that.positions);
        that.floatLabel.label.text = "方位角：" + that.azimutht.toFixed(2);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.lastPosition = null;
        that.nextPosition = null;
        that.forbidDrawWorld(false);
        if (callback) callback();
        that.state = "endEdit";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
    /**
     * 结束编辑
     */

  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //sin 2024.7.17 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    }
    /**
    * 销毁
    */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      if (this.floatLabel) {
        this.viewer.entities.remove(this.floatLabel);
        this.floatLabel = null;
      }

      this.floatLable = null;

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      this.state = "no";

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }
    }
  }]);

  return MeasureAzimutht;
}(BaseMeasure);

/**
 * 剖面测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureSection 
 */

var MeasureSection = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureSection, _BaseMeasure);

  var _super = _createSuper(MeasureSection);

  function MeasureSection(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureSection);

    _this = _super.call(this, viewer, opt);
    _this.style = opt.style || {};
    _this.viewer = viewer; //线

    _this.polyline = null; //线坐标

    _this.positions = []; //标签数组

    _this.movePush = false;
    _this.prompt;
    _this.isStart = false;
    _this.firstPosition = null;
    _this.state = "no";
    return _this;
  } //开始测量


  _createClass(MeasureSection, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt(this.viewer, this.promptStyle);
      var that = this;
      that.state = "startCreate";
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (!that.isStart) {
          that.isStart = true;
          that.firstPosition = cartesian;
        } else {
          if (that.handler) {
            that.handler.destroy();
            that.handler = null;
          }

          if (that.prompt) {
            that.prompt.destroy();
            that.prompt = null;
          } // 生成剖面图数据


          that.getHeight(that.positions, function (data) {
            callback(data);
          });
          that.state = "endCreate";
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        //移动时绘制线
        that.state = "creating";

        if (!that.isStart) {
          that.prompt.update(evt.endPosition, "单击开始");
          return;
        }

        that.prompt.update(evt.endPosition, "再次单击结束");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.positions = [that.firstPosition, cartesian];

        if (!that.polyline) {
          that.polyline = that.viewer.entities.add({
            polyline: {
              show: true,
              positions: new Cesium.CallbackProperty(function () {
                return that.positions;
              }, false),
              material: Cesium.Color.GREEN,
              width: 3,
              clampToGround: true
            }
          });
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      this.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      this.viewer.trackedEntity = undefined;
      this.state = "no";
    }
  }, {
    key: "getHeight",
    value: function getHeight(positions, callback) {
      if (!positions || positions.length < 1) return; // 求出该点周围两点的坐标 构建平面

      positions = util.lerpPositions(positions);
      var ctgs = [];
      positions.forEach(function (item) {
        ctgs.push(Cesium.Cartographic.fromCartesian(item));
      });
      if (!ctgs || ctgs.length < 1) return;
      var first = Cesium.Cartographic.fromCartesian(positions[0]);
      var height = first.height;
      Cesium.sampleTerrainMostDetailed(this.viewer.terrainProvider, ctgs).then(function (updateLnglats) {
        for (var i = 0; i < updateLnglats.length; i++) {
          var item = updateLnglats[i];
          item.height = item.height ? item.height : height;
        }

        if (callback) callback({
          positions: positions,
          lnglats: updateLnglats
        });
      });
    }
  }]);

  return MeasureSection;
}(BaseMeasure);

/**
 * 坡度测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureSlope 
 */

var MeasureSlope = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureSlope, _BaseMeasure);

  var _super = _createSuper(MeasureSlope);

  function MeasureSlope(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureSlope);

    if (!opt) opt = {};
    _this = _super.call(this, viewer, opt);
    _this.style = opt.style || {};
    _this.viewer = viewer;
    _this.label = null;
    _this.point = null;
    return _this;
  } //开始测量


  _createClass(MeasureSlope, [{
    key: "start",
    value: function start() {
      this.state = "startCreate";
      var that = this;
      this.handler.setInputAction(function (evt) {
        //单击开始绘制
        if (that.handler) {
          that.handler.destroy();
          that.handler = null;
          that.state = "endCreate";
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (!that.point) {
          that.point = that.createPoint(cartesian);
        }

        that.point.position.setValue(cartesian);
        that.getSlope(cartesian, function (slop) {
          if (!that.label) that.label = that.createLabel(cartesian, "");
          that.label.position.setValue(cartesian);
          that.label.label.text = "坡度：" + slop.toFixed(2) + "°";
        });
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      } else {
        this.endCreate();
      }
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      this.state = "no";

      if (this.label) {
        this.viewer.entities.remove(this.label);
        this.label = null;
      }

      if (this.point) {
        this.viewer.entities.remove(this.point);
        this.point = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }
    }
  }, {
    key: "createPoint",
    value: function createPoint(position) {
      var _point;

      return this.viewer.entities.add({
        position: position,
        point: (_point = {
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
          show: true,
          fillColor: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.RED,
          outlineWidth: 2,
          pixelSize: 6
        }, _defineProperty(_point, "outlineWidth", 3), _defineProperty(_point, "disableDepthTestDistance", Number.MAX_VALUE), _point)
      });
    }
  }]);

  return MeasureSlope;
}(BaseMeasure);

/**
 * 空间面积测量类
 * @class
 * @augments BaseMeasure
 * @alias BaseMeasure.MeasureSpaceArea 
 */

var MeasureSpaceArea$1 = /*#__PURE__*/function (_BaseMeasure) {
  _inherits(MeasureSpaceArea, _BaseMeasure);

  var _super = _createSuper(MeasureSpaceArea);

  function MeasureSpaceArea(viewer, opt) {
    var _this;

    _classCallCheck(this, MeasureSpaceArea);

    _this = _super.call(this, viewer, opt);
    if (!opt) opt = {};
    _this.unitType = "area";
    _this.unit = opt.unit || "平方米";
    _this.style = opt.style || {};
    _this.viewer = viewer;
    _this.polyline = null;
    _this.polygon = null; //面积标签

    _this.positions = [];
    _this.movePush = false;
    _this.prompt = undefined;
    _this.geometry = undefined;
    return _this;
  } //开始测量


  _createClass(MeasureSpaceArea, [{
    key: "start",
    value: function start(callback) {
      if (!this.prompt && this.promptStyle.show) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      var that = this;
      this.state = "startCreate";
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (that.movePush) {
          that.positions.pop();
          that.movePush = false;
        }

        var point = that.createPoint(cartesian.clone());
        point.wz = that.positions.length; // 和坐标点关联

        that.controlPoints.push(point);
        that.positions.push(cartesian);
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";

        if (that.positions.length < 1) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          return;
        }else if(that.positions.length==2){
          that.prompt.update(evt.endPosition, "请选择第二个点");
        }else if(that.positions.length>2){
          that.prompt.update(evt.endPosition, "双击结束，右键取消上一步");
        }
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        if (that.positions.length >= 1) {
          if (!that.movePush) {
            that.positions.push(cartesian);
            that.movePush = true;
          } else {
            that.positions[that.positions.length - 1] = cartesian;
          }

          if (that.positions.length == 2) {
            if (!Cesium.defined(that.polyline)) {
              that.polyline = that.createPolyline();
            }
          }

          if (that.positions.length >= 3) {
            if (!Cesium.defined(that.polygon)) {
              that.polygon = that.createPolygon();
              that.polygon.isFilter = true;
              that.polygon.objId = that.objId;
            }

            if (!that.floatLabel) {
              that.floatLabel = that.createLabel(cartesian, "");
              // that.floatLabel.label.heightReference = 1;
            }
          }

          if (that.polygon) {
            that.getGroundAreaAndCenter(that.positions, function (area, center) {
              var text = that.formateArea(area, that.unit);
              that.floatLabel.label.text = "面积：" + text;
              that.floatLabel.area = area;
              if (center) that.floatLabel.position.setValue(center);
              that.floatLabel.show = true;
            });
          }
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (evt) {
        that.state = "creating";
        if (!that.polyline && !that.polygon) return;
        that.positions.splice(that.positions.length - 2, 1);
        that.viewer.entities.remove(that.controlPoints.pop());

        if (that.positions.length == 2) {
          if (that.polygon) {
            that.viewer.entities.remove(that.polygon);
            that.polygon = null;
            if (that.polyline) that.polyline.show = true;
          }

          that.floatLabel.show = false;
        }

        if (that.positions.length == 1) {
          if (that.polyline) {
            that.viewer.entities.remove(that.polyline);
            that.polyline = null;
          }

          that.prompt.update(evt.endPosition, "单击开始测量");
          that.positions = [];
          that.movePush = false;
        }

        if (that.positions.length > 2) {
          that.getGroundAreaAndCenter(that.positions, function (area, center) {
            var text = that.formateArea(area, that.unit);
            that.floatLabel.label.text = "面积：" + text;
            if (center) that.floatLabel.position.setValue(center);
            that.floatLabel.area = area;
            that.floatLabel.show = true;
          });
        }
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
      this.handler.setInputAction(function (evt) {
        //双击结束绘制
        if (!that.polygon) {
          return;
        }

        that.positions.pop();
        that.viewer.entities.remove(that.controlPoints.pop()); // 移除最后一个

        that.movePush = false;
        that.endCreate();
        if (callback) callback(that.polyline);
      }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    }
  }, {
    key: "endCreate",
    value: function endCreate() {
      var that = this;
      that.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      that.viewer.trackedEntity = undefined;
      that.getGroundAreaAndCenter(that.positions, function (area, center) {
        var text = that.formateArea(area, that.unit);
        that.floatLabel.label.text = "面积：" + text;
        that.floatLabel.area = area;
        if (center) that.floatLabel.position.setValue(center);
      });

      if (that.handler) {
        that.handler.destroy();
        that.handler = null;
      }

      if (that.prompt) {
        that.prompt.destroy();
        that.prompt = null;
      }

      that.state = "endCreate";
    }
  }, {
    key: "done",
    value: function done() {
      if (this.state == "startCreate") {
        this.destroy();
      } else if (this.state == "creating") {
        if (this.positions.length <= 2 && this.movePush == true) {
          this.destroy();
        } else {
          this.endCreate();
        }
      } else if (this.state == "startEdit" || this.state == "editing") {
        this.endEdit();
      }
    }
  }, {
    key: "startEdit",
    value: function startEdit(callback) {
      if (!((this.state == "endCreate" || this.state == "endEdit") && this.polygon)) return;
      this.state = "startEdit";
      if (!this.modifyHandler) this.modifyHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      var that = this;

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = true;
      }

      this.modifyHandler.setInputAction(function (evt) {
        var pick = that.viewer.scene.pick(evt.position);

        if (Cesium.defined(pick) && pick.id && pick.primitive instanceof Cesium.PointPrimitive) {
          if (!pick.id.objId) that.modifyPoint = pick.id;
          that.forbidDrawWorld(true);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.modifyHandler.setInputAction(function (evt) {
        if (that.positions.length < 1 || !that.modifyPoint) return;
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;
        that.modifyPoint.position.setValue(cartesian);
        var wz = that.modifyPoint.wz;
        that.positions[wz] = cartesian.clone();
        that.getGroundAreaAndCenter(that.positions, function (area, center) {
          var text = that.formateArea(area, that.unit);
          that.floatLabel.label.text = "面积：" + text;
          that.floatLabel.area = area;
          if (center) that.floatLabel.position.setValue(center);
        });
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.modifyHandler.setInputAction(function (evt) {
        if (!that.modifyPoint) return;
        that.modifyPoint = null;
        that.forbidDrawWorld(false);
        if (callback) callback();
        that.state = "endEdit";
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
    }
  }, {
    key: "endEdit",
    value: function endEdit() {
      var that = this;
      this.state = "endEdit";

      if (this.modifyHandler) {
        this.modifyHandler.destroy();
        this.modifyHandler = null;
      }

      for (var i = 0; i < that.controlPoints.length; i++) {
        var point = that.controlPoints[i];
        if (point) point.show = false;
      }
      //sin 2024.7.17 bug
      that.forbidDrawWorld(false);
      that.modifyPoint = null;
    } //清除测量结果

  }, {
    key: "destroy",
    value: function destroy() {
      this.state = "no";

      if (this.polyline) {
        this.viewer.entities.remove(this.polyline);
        this.polyline = null;
      }

      if (this.polygon) {
        this.viewer.entities.remove(this.polygon);
        this.polygon = null;
      }

      if (this.floatLabel) {
        this.viewer.entities.remove(this.floatLabel);
        this.floatLabel = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      for (var i = 0; i < this.controlPoints.length; i++) {
        var point = this.controlPoints[i];
        this.viewer.entities.remove(point);
      }

      this.floatLable = null;
    }
  }, {
    key: "createPolyline",
    value: function createPolyline() {
      var that = this;
      var polyline = this.viewer.entities.add({
        polyline: {
          positions: new Cesium.CallbackProperty(function () {
            var linePositions = that.positions.concat([that.positions[0]]);
            return linePositions;
          }, false),
          material: Cesium.Color.GOLD,
          width: 2,
          clampToGround: true
        }
      });
      return polyline;
    }
  }, {
    key: "createPolygon",
    value: function createPolygon() {
      var that = this;
      var polygon = this.viewer.entities.add({
        polygon: new Cesium.PolygonGraphics({
          hierarchy: new Cesium.CallbackProperty(function () {
            return new Cesium.PolygonHierarchy(that.positions);
          }, false),
          material: this.style.material || Cesium.Color.WHITE.withAlpha(0.6),
          heightReference: 1,
          fill: true
        })
      });
      return polygon;
    }
  }, {
    key: "setUnit",
    value: function setUnit(unit) {
      this.unit = unit;
      var text = this.formateArea(this.floatLabel.area, unit);
      this.floatLabel.label.text = "面积：" + text;
    } // 计算贴地面积 异步

  }, {
    key: "getGroundAreaAndCenter",
    value: function getGroundAreaAndCenter(positions, callback) {
      var _this2 = this;

      if (!positions || positions.length < 2) return;
      var area = util$1.computeArea(positions, this.viewer);
      var polygonGeometry = new Cesium.PolygonGeometry.fromPositions({
        positions: positions,
        vertexFormat: Cesium.PerInstanceColorAppearance.FLAT_VERTEX_FORMAT,
        granularity: Math.PI / Math.pow(2, 11) / 1000 * (area / 10)
      });
      var geom = new Cesium.PolygonGeometry.createGeometry(polygonGeometry);
      var indices = geom.indices;
      var attrPosition = geom.attributes.position;
      var arr = [];
      var ctgcs = [];

      for (var index = 0; index < indices.length; index = index + 3) {
        var first = indices[index];
        var second = indices[index + 1];
        var third = indices[index + 2];
        var cartesian1 = new Cesium.Cartesian3(attrPosition.values[first * 3], geom.attributes.position.values[first * 3 + 1], attrPosition.values[first * 3 + 2]);
        var ctgc1 = Cesium.Cartographic.fromCartesian(cartesian1.clone());
        arr.push(cartesian1.clone());
        ctgcs.push(ctgc1);
        var cartesian2 = new Cesium.Cartesian3(attrPosition.values[second * 3], geom.attributes.position.values[second * 3 + 1], attrPosition.values[second * 3 + 2]);
        var ctgc2 = Cesium.Cartographic.fromCartesian(cartesian2.clone());
        arr.push(cartesian2.clone());
        ctgcs.push(ctgc2);
        var cartesian3 = new Cesium.Cartesian3(geom.attributes.position.values[third * 3], geom.attributes.position.values[third * 3 + 1], attrPosition.values[third * 3 + 2]);
        var ctgc3 = Cesium.Cartographic.fromCartesian(cartesian3.clone());
        arr.push(cartesian3.clone());
        ctgcs.push(ctgc3);
      } // 计算贴地高度

      console.log(22, this.viewer.terrainProvider, ctgcs)
      // var updatedPositions = Cesium.sampleTerrainMostDetailed(this.viewer.terrainProvider, ctgcs);
      var updatedPositions = Cesium.sampleTerrain(this.viewer.terrainProvider,9,ctgcs);
      
      updatedPositions.then(function (newCtgcs) {
        var area = 0;

        var center = _this2.getCenter(positions);

        center = Cesium.Cartesian3.fromDegrees(center[0], center[1]);

        for (var ind = 0; ind < newCtgcs.length; ind += 3) {
          var c1 = newCtgcs[ind];
          var c2 = newCtgcs[ind + 1];
          var c3 = newCtgcs[ind + 2];
          var cart1 = Cesium.Cartographic.toCartesian(c1);
          var cart2 = Cesium.Cartographic.toCartesian(c2);
          var cart3 = Cesium.Cartographic.toCartesian(c3);
          var res = util$1.computeAreaOfTriangle(cart1, cart2, cart3);
          area += res;
        }

        if (callback) callback(area, center);
      });
    } // 计算当前面的中心点

  }, {
    key: "getCenter",
    value: function getCenter(positions) {
      if (!positions || positions.length < 1) return;
      var cartographics = [];
      var turfPoints = [];

      for (var i = 0; i < positions.length; i++) {
        var cartesian3 = positions[i];
        var cartographic = Cesium.Cartographic.fromCartesian(cartesian3);
        cartographics.push([Cesium.Math.toDegrees(cartographic.longitude), Cesium.Math.toDegrees(cartographic.latitude)]);
        turfPoints.push(point([Cesium.Math.toDegrees(cartographic.longitude), Cesium.Math.toDegrees(cartographic.latitude)]));
      }

      if (!cartographics.length) return;
      cartographics = cartographics.concat([cartographics[0]]); //获取当前范围的中心点

      var features = featureCollection(turfPoints);
      var turfCenter = center(features);
      var center$1 = turfCenter.geometry.coordinates;
      return center$1;
    }
  }]);

  return MeasureSpaceArea;
}(BaseMeasure);

/**
 * 量算控制类
 * @description 量算控制类，通过此类对象，可进行不同类型的量算操作，而不用多次new 不同类型的量算对象。
 * @class
 */

var MeasureTool = /*#__PURE__*/function () {
  /**
   * @param {Cesium.viewer} viewer 地图viewer对象
   * @param {Object} obj 基础配置 
   */
  function MeasureTool(viewer, obj) {
    _classCallCheck(this, MeasureTool);

    if (!viewer) {
      console.warn("缺少必要参数！--viewer");
      return;
    }

    obj = obj || {};
    this.viewer = viewer;
    /**
     * @property {Object} nowDrawMeasureObj 当前测量对象
     */

    this.nowDrawMeasureObj = null;
    /**
     * @property {Array} measureObjArr 测量对象数组
     */

    this.measureObjArr = [];
    this.nowEditMeasureObj = null;
    this.handler = null;
    /**
     * @property {Boolean} [canEdit=true] 测量对象是否可编辑
     */

    this.canEdit = obj.canEdit == undefined ? true : obj.canEdit;
    /**
     * @property {Boolean} [intoEdit=true] 绘制完成后，是否进入编辑状态（当canEdit==true，才起作用）
     */

    this.intoEdit = obj.intoEdit == undefined ? true : obj.intoEdit;
    this.bindEdit();
    /**
     * @property {Object} nowDrawMeasureObj 当前绘制对象，绘制完成后为undifined
     */

    this.nowDrawMeasureObj = undefined;
    /**
     * @property {Object} nowEditMeasureObj 当前编辑对象，编辑完成后为undifined
     */

    this.nowEditMeasureObj = undefined;
  }
  /** 
   * 事件绑定
   * @param {String} type 事件类型（startEdit 开始编辑时 / endEdit 编辑结束时 / endCreate 创建完成后）
   * @param {Function} fun 绑定函数
   */


  _createClass(MeasureTool, [{
    key: "on",
    value: function on(type, fun) {
      if (type == "endCreate") {
        this.endCreateFun = fun;
      }

      if (type == "startEdit") {
        this.startEditFun = fun;
      }

      if (type == "endEdit") {
        this.endEditFun = fun;
      }
    }
    /**
     * 开始量算
     * @param {Object} opt 
     * @param {Number} opt.type 量算类型（1~空间距离测量/2~贴地距离测量/3~空间面积测量/4~高度测量/5~三角测量/6~坐标量算/7~方位角测量/8~剖面测量/9~单点坡度）
     */

  }, {
    key: "start",
    value: function start(opt) {
      opt = opt || {};
      if (!opt.type) return;
      var ms;
      this.endEdit();
      if (this.nowDrawMeasureObj && this.nowDrawMeasureObj.state != "endCreate" && this.nowDrawMeasureObj.state != "endEdit" && this.nowDrawMeasureObj.state != "no") return;

      switch (Number(opt.type)) {
        case 1:
          // 空间距离测量
          ms = new MeasureSpaceDistance(this.viewer, opt);
          break;

        case 2:
          // 贴地距离测量
          ms = new MeasureGroundDistance(this.viewer, opt);
          break;

        case 3:
          // 空间面积测量
          ms = new MeasureSpaceArea(this.viewer, opt);
          break;

        case 4:
          // 高度测量
          ms = new MeasureHeight(this.viewer, opt);
          break;

        case 5:
          // 三角测量
          ms = new MeasureTriangle(this.viewer, opt);
          break;

        case 6:
          // 坐标量算
          ms = new MeasureLnglat(this.viewer, opt);
          break;

        case 7:
          // 方位角测量
          ms = new MeasureAzimutht(this.viewer, opt);
          break;

        case 8:
          // 剖面测量
          ms = new MeasureSection(this.viewer, opt);
          break;

        case 9:
          // 单点坡度
          ms = new MeasureSlope(this.viewer, opt);
          break;

        /* 	case 10: //贴模型距离
        		ms = new MeasureTilesetDistance(this.viewer);
        		break; */

        case 11:
          // 单点坡度
          ms = new MeasureSlopePolygon(this.viewer);
          break;

        case 12:
          // 贴地面积测量
          ms = new MeasureSpaceArea$1(this.viewer, opt);
          break;
      }

      this.nowDrawMeasureObj = ms;
      this.nowDrawMeasureObj.attr = opt || {}; // 保存开始绘制时的属性

      var that = this;

      if (ms) {
        this.changeCursor(true);
        ms.start(function (res) {
          that.changeCursor(false);

          if (that.intoEdit) {
            ms.startEdit();
            that.nowEditMeasureObj = ms;
            if (that.startEditFun) that.startEditFun(ms);
          }

          if (opt.success) opt.success(ms, res);
          if (that.endCreateFun) that.endCreateFun(ms, res);
          that.nowDrawMeasureObj = undefined;
          that.measureObjArr.push(ms);
        });
      }

      return ms;
    }
    /**
     * 绑定编辑
     */

  }, {
    key: "bindEdit",
    value: function bindEdit() {
      var that = this; // 如果是线 面 则需要先选中

      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        if (!that.canEdit) return; // 若当前正在绘制 则无法进行编辑操作

        if (that.nowDrawMeasureObj) return;
        var pick = that.viewer.scene.pick(evt.position);
        if (Cesium.defined(pick) && pick.id && pick.id.objId && pick.primitive instanceof Cesium.PointPrimitive) {
          // 选中实体 
          for (var i = 0; i < that.measureObjArr.length; i++) {
            if (pick.id.objId == that.measureObjArr[i].objId && (that.measureObjArr[i].state == "endCreate" || that.measureObjArr[i].state == "endEdit")) {
              // 结束上一个编辑
              if (that.nowEditMeasureObj) {
                // 结束除当前选中实体的所有编辑操作
                that.nowEditMeasureObj.endEdit();
                if (that.endEditFun) that.endEditFun(that.nowEditMeasureObj);
                that.nowEditMeasureObj = undefined;
              } // 开始当前编辑


              that.measureObjArr[i].startEdit();
              that.nowEditMeasureObj = that.measureObjArr[i];
              if (that.startEditFun) that.startEditFun(that.nowEditMeasureObj); // 开始编辑

              break;
            }
          }
        } else {
          // 未选中实体 则结束编辑
          if (that.nowEditMeasureObj) {
            that.nowEditMeasureObj.endEdit();
            if (that.endEditFun) that.endEditFun(that.nowEditMeasureObj); // 结束事件

            that.nowEditMeasureObj = undefined;
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
    }
    /**
     * 结束的当前操作
     */

  }, {
    key: "done",
    value: function done() {
      if (this.nowEditMeasureObj) {
        this.nowEditMeasureObj.done();
        if (this.endEditFun) this.endEditFun(this.nowEditMeasureObj);
        this.nowEditMeasureObj = undefined;
      }

      if (this.nowDrawMeasureObj) {
        this.nowDrawMeasureObj.done();
        this.measureObjArr.push(this.nowDrawMeasureObj);
        if (this.endCreateFun) this.endCreateFun(this.nowDrawMeasureObj);
        this.nowDrawMeasureObj = undefined;
      }
    }
    /**
     * 结束编辑
     */

  }, {
    key: "endEdit",
    value: function endEdit() {
      if (this.nowEditMeasureObj) {
        // 结束除当前选中实体的所有编辑操作
        this.nowEditMeasureObj.endEdit();
        if (this.endEditFun) this.endEditFun(this.nowEditMeasureObj); // 结束事件

        this.nowEditMeasureObj = null;
      }

      for (var i = 0; i < this.measureObjArr.length; i++) {
        this.measureObjArr[i].endEdit();
      }
    }
    /**
     * 清除
     */

  }, {
    key: "clear",
    value: function clear() {
      for (var i = 0; i < this.measureObjArr.length; i++) {
        if (this.measureObjArr[i]) {
          this.measureObjArr[i].endEdit();
          this.measureObjArr[i].destroy();
        }
      }

      this.measureObjArr = [];

      if (this.nowDrawMeasureObj) {
        this.nowDrawMeasureObj.destroy();
        this.nowDrawMeasureObj = null; // 当前编辑对象
      }

      this.changeCursor(false);
    }
    /**
     * 销毁
    */

  }, {
    key: "destroy",
    value: function destroy() {
      this.clear();

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }
    }
    /**
     * 设置单位
    */

  }, {
    key: "setMeasureObjUnit",
    value: function setMeasureObjUnit(item, unit) {
      if (!item || !unit) return;
      item.setUnit(unit);
    }
    /**
     * 修改鼠标样式
     * @param {Boolean} isopen false为默认鼠标样式
    */

  }, {
    key: "changeCursor",
    value: function changeCursor(isopen) {
      var body = document.getElementsByTagName("body");
      body[0].style.cursor = isopen ? "crosshair" : "default";
    }
    /**
     * 根据id获取量算对象
     * @param {*} id 
     * @returns {Object} measureObj为图层对象，index为图层对象在数组中位置
     */

  }, {
    key: "getMeasureObjById",
    value: function getMeasureObjById(id) {
      if (!id) return;
      var res = {};

      for (var i = 0; i < this.measureObjArr.length; i++) {
        if (this.measureObjArr[i].attr.id == id) {
          res = {
            measureObj: this.measureObjArr[i],
            index: i
          };
          break;
        }
      }

      return res;
    }
    /**
     * 根据objId获取量算对象
     * @param {*} id 
     * @returns {Object} measureObj为图层对象，index为图层对象在数组中位置
     */

  }, {
    key: "getMeasureObjByObjId",
    value: function getMeasureObjByObjId(id) {
      if (!id) return;
      var res = {};

      for (var i = 0; i < this.measureObjArr.length; i++) {
        if (this.measureObjArr[i].objId == id) {
          res = {
            measureObj: this.measureObjArr[i],
            index: i
          };
          break;
        }
      }

      return res;
    }
    /**
     * 删除单个量算对象
     * @param {Object} measureObj 
     */

  }, {
    key: "removeOne",
    value: function removeOne(measureObj) {
      if (!measureObj) return;
      var res = this.getMeasureObjByObjId(measureObj.objId);

      if (res.measureObj) {
        this.measureObjArr.splice(res.index, 1);
        res.measureObj.destroy();
      }
    }
  }]);

  return MeasureTool;
}();

/**
 * 漫游类
 * @class
 *
 */

var Roam = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象
   * @param {Object} opt
   * @param {Date} [opt.startTime] 漫游开始时间
   * @param {Cesium.Cartesian3[] | Array} [opt.positions] 路线坐标
   * @param {Functioin} [opt.endRoam] 结束漫游时的回调函数
   * @param {Functioin} [opt.roaming] 漫游过程中的回调函数
   * @param {Number} [opt.alltimes] 漫游总时长（s）
   * @param {Number} [opt.speed] 漫游速度（m/s）
   */
  function Roam(viewer, opt) {
    _classCallCheck(this, Roam);

    this.viewer = viewer;
    /**
     * @property {Number} objId 唯一标识
     */

    this.objId = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    this.opt = opt || {};
    /**
     * @property {Cesium.JulianDate} startTime 开始时间
     */

    this.startTime = opt.startTime ? Cesium.JulianDate.fromDate(opt.startTime, new Cesium.JulianDate()) : this.viewer.clock.currentTime.clone();
    /**
     * @property {Cesium.JulianDate} endTime 结束时间
     */

    this.endTime = null;

    if (!this.opt.positions) {
      console.log("缺少漫游坐标");
      return;
    }
    /**
     *  @property {Cesium.Cartesian3[]} positions 漫游坐标
     */


    this.positions = this.transfromPositions(this.opt.positions);
    /**
     * @property {Number} clockSpeed 播放速度
     */

    this.clockSpeed = 1;
    /**
     * @property {Cesium.JulianDate} stopTime 暂停时间
     */

    this.stopTime = null;
    /**
     * @property {Number} alldistance 漫游总距离
     */

    this.alldistance = 0;
    /**
     * @property {Number} alltimes 漫游时间
     */

    this.alltimes = 0;
    /**
     * @property {Number} distanceED 已漫游距离
     */

    this.distanceED = -1;
    /**
     * @property {Number} timesED 已漫游时间
     */

    this.timesED = -1;
    /**
     * @property {Number} speed 漫游速度
     */

    this.speed = 0;
    /**
     * @property {String} viewType 漫游视角（no~不固定视角/gs~跟随视角/dy~第一视角/sd~上帝视角）
     */

    this.viewType = "no";
    this.rendHandler = null;
    /**
     * @property {Boolean} isLockView 是否锁定视角
     */

    this.isLockView = false;
    this.viewXYZ = {
      // 锁定时视角参数
      x: 0,
      y: 0,
      z: 0
    };
    this.endRoam = opt.endRoam;
    this.roaming = opt.roaming;
    this.fixType = this.opt.alltimes ? "0" : "1"; // 0 表示固定时长漫游 1 表示固定速度漫游

    this.init();
    this.setViewType(opt.viewType); // 初始化时 设置视角
  }

  _createClass(Roam, [{
    key: "init",
    value: function init() {
      var attr = {};

      if (this.fixType == "0") {
        // 固定时长漫游
        this.endTime = Cesium.JulianDate.addSeconds(this.startTime, Number(this.opt.alltimes), new Cesium.JulianDate());
        attr = this.createPropertyByTimes(this.positions, this.opt.alltimes);
      } else {
        // 固定速度漫游 (m/s)
        if (!this.opt.speed) {
          console.log("缺少漫游时长或速度参数！");
          return;
        }

        attr = this.createPropertyBySpeed(this.positions, this.opt.speed);
      }

      this.alldistance = attr.alldistance;
      this.alltimes = attr.alltimes;
      this.speed = attr.speed;
      this.roamEntity = this.createRoamEntity(this.opt.entityType, attr.property);
    } // 修改漫游的路径

    /**
     *
     * @param {Cesium.Cartesian3[] | Array} positions 路线坐标
     */

  }, {
    key: "setPositions",
    value: function setPositions(positions) {
      this.destroy();
      this.positions = positions;
      this.init();
    }
    /**
     * 开始漫游
     */

  }, {
    key: "start",
    value: function start() {
      if (this.roamEntity) this.roamEntity.show = true;
      this.clockSpeed = this.clockSpeed || 1;
      this.viewer.clock.currentTime = this.startTime;
      this.viewer.clock.multiplier = this.clockSpeed || 1;
      this.viewer.clock.shouldAnimate = true;
      this.viewer.clock.clockRange = Cesium.ClockRange.LOOP_STOP;
      this.computeCamera(); // 设置视角
    }
    /**
     * 结束漫游
     */

  }, {
    key: "end",
    value: function end() {
      if (this.roamEntity) this.roamEntity.show = false;
      this.viewer.clock.currentTime = this.endTime;
      this.viewer.clock.shouldAnimate = false;
      this.distanceED = this.alldistance;
      this.timesED = this.alltimes;
      this.viewer.trackedEntity = undefined;
      this.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);

      if (this.rendHandler) {
        this.rendHandler();
        this.rendHandler = null;
      }

      if (this.endRoam) this.endRoam(this.opt);
    }
    /**
     * 暂停漫游
     */

  }, {
    key: "stop",
    value: function stop() {
      this.stopTime = this.viewer.clock.currentTime.clone();
      this.viewer.clock.shouldAnimate = false; // if (this.roamingFun) this.roamingFun();
    }
    /**
    * 继续漫游
    */

  }, {
    key: "goon",
    value: function goon() {
      if (!this.stopTime) return;
      this.viewer.clock.currentTime = this.stopTime.clone();
      this.viewer.clock.shouldAnimate = true;
      this.stopTime = null;
    }
    /**
     * 设置播放速度
     * @param {Number} speed 播放速度
     */

  }, {
    key: "setSpeed",
    value: function setSpeed(speed) {
      this.clockSpeed = speed;
      this.viewer.clock.multiplier = this.clockSpeed;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.roamEntity) {
        this.viewer.entities.remove(this.roamEntity);
        this.roamEntity = null;
      }

      if (this.rendHandler) {
        this.rendHandler();
        this.rendHandler = null;
      }

      this.viewer.clock.multiplier = 1;
      this.isLockView = false;
      this.viewer.trackedEntity = undefined;
      this.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    }
  }, {
    key: "createRoamEntity",
    value: function createRoamEntity(type, property) {
      var entity = null;

      if (type == "model") {
        if (!this.opt.entityAttr || !this.opt.entityAttr.uri) {
          console.log("漫游缺少模型对象！");
          return;
        }

        entity = this.viewer.entities.add({
          orientation: new Cesium.VelocityOrientationProperty(property),
          position: property,
          model: this.opt.entityAttr
        });
      } else if (type == "image") {
        if (!this.opt.entityAttr || !this.opt.entityAttr.image) {
          console.log("漫游缺少图片对象！");
          return;
        }

        entity = this.viewer.entities.add({
          orientation: new Cesium.VelocityOrientationProperty(property),
          position: property,
          billboard: this.opt.entityAttr
        });
      } else {
        entity = this.viewer.entities.add({
          orientation: new Cesium.VelocityOrientationProperty(property),
          position: property,
          point: {
            pixelSize: 0.001,
            color: Cesium.Color.WHITE.withAlpha(0.0001)
          }
        });
      }

      entity.show = false;
      return entity;
    }
  }, {
    key: "transfromPositions",
    value: function transfromPositions(positions) {
      if (!positions || positions.length < 1) return;

      if (positions[0] instanceof Cesium.Cartesian3) {
        return positions;
      } else {
        var newPositions = [];
        positions.forEach(function (element) {
          var p = Cesium.Cartesian3.fromDegrees(element[0], element[1], element[2] || 0);
          newPositions.push(p);
        });
        return newPositions;
      }
    }
  }, {
    key: "reversePositions",
    value: function reversePositions(positions) {
      if (!positions || positions.length < 1) return;

      if (positions[0] instanceof Cesium.Cartesian3) {
        return util$1.cartesiansToLnglats(positions, this.viewer);
      } else {
        return positions;
      }
    } // 实时计算相机视角

  }, {
    key: "computeCamera",
    value: function computeCamera() {
      var that = this;
      var scratch = new Cesium.Matrix4();
      this.distanceED = 0; // 飞过的距离

      this.timeED = 0; // 所用时间

      var lastPosition = null;

      if (!this.rendHandler) {
        this.rendHandler = this.viewer.scene.preRender.addEventListener(function (e) {
          if (!that.viewer.clock.shouldAnimate || !that.roamEntity) return;
          var currentTime = that.viewer.clock.currentTime;
          var tiemC = Cesium.JulianDate.compare(that.endTime, currentTime);

          if (tiemC < 0) {
            that.end();
            return;
          }

          if (that.roaming) that.roaming(that.distanceED, that.timesED);

          if (that.isLockView) {
            that.getModelMatrix(that.roamEntity, that.viewer.clock.currentTime, scratch);
            that.viewer.scene.camera.lookAtTransform(scratch, new Cesium.Cartesian3(-that.viewXYZ.x, that.viewXYZ.y, that.viewXYZ.z));
          }

          that.timeED = Cesium.JulianDate.secondsDifference(currentTime, that.startTime);
          var position = that.roamEntity.position.getValue(currentTime);

          if (position && lastPosition) {
            that.distanceED += Cesium.Cartesian3.distance(position, lastPosition);
          }

          lastPosition = position;
        });
      }
    }
  }, {
    key: "getModelMatrix",
    value: function getModelMatrix(entity, time, result) {
      if (!entity) return;
      var position = Cesium.Property.getValueOrUndefined(entity.position, time, new Cesium.Cartesian3());
      if (!Cesium.defined(position)) return;
      var orientation = Cesium.Property.getValueOrUndefined(entity.orientation, time, new Cesium.Quaternion());

      if (!orientation) {
        result = Cesium.Transforms.eastNorthUpToFixedFrame(position, undefined, result);
      } else {
        result = Cesium.Matrix4.fromRotationTranslation(Cesium.Matrix3.fromQuaternion(orientation, new Cesium.Matrix3()), position, result);
      }

      return result;
    } // 构建漫游的property

  }, {
    key: "createPropertyByTimes",
    value: function createPropertyByTimes(positions, times) {
      if (!positions || positions.length < 2) return;
      var property = new Cesium.SampledPositionProperty();
      var alldistance = 0; // 总距离

      for (var i = 1; i < positions.length; i++) {
        var p = positions[i - 1];
        var nextP = positions[i];
        var distance = Cesium.Cartesian3.distance(p, nextP);
        alldistance += distance;
      }

      var speed = alldistance / times; // 速度

      var passdistance = 0;

      for (var ind = 0; ind < positions.length; ind++) {
        var nowP = positions[ind];
        var currentTime = void 0;

        if (ind == 0) {
          currentTime = this.startTime.clone();
        } else {
          var lastP = positions[ind - 1];

          var _distance = Cesium.Cartesian3.distance(nowP, lastP);

          passdistance += _distance;

          var _times = passdistance / speed;

          currentTime = Cesium.JulianDate.addSeconds(this.startTime.clone(), Number(_times), new Cesium.JulianDate());
        }

        property.addSample(currentTime.clone(), nowP.clone());
      }

      return {
        property: property,
        alldistance: alldistance,
        alltimes: times,
        speed: speed
      };
    }
  }, {
    key: "createPropertyBySpeed",
    value: function createPropertyBySpeed(positions, speed) {
      if (!positions || positions.length < 2) return;
      var property = new Cesium.SampledPositionProperty();
      var alldistance = 0; // 总距离

      for (var i = 1; i < positions.length; i++) {
        var p = positions[i - 1];
        var nextP = positions[i];
        var distance = Cesium.Cartesian3.distance(p, nextP);
        alldistance += distance;
      }

      var passdistance = 0;

      for (var ind = 0; ind < positions.length; ind++) {
        var nowP = positions[ind];
        var currentTime = void 0;

        if (ind == 0) {
          currentTime = this.startTime.clone();
        } else {
          var lastP = positions[ind - 1];

          var _distance2 = Cesium.Cartesian3.distance(nowP, lastP);

          passdistance += _distance2;
          var times = passdistance / speed;
          currentTime = Cesium.JulianDate.addSeconds(this.startTime.clone(), Number(times), new Cesium.JulianDate());
        }

        property.addSample(currentTime.clone(), nowP.clone());
      }

      return {
        property: property,
        alldistance: alldistance,
        alltimes: alldistance / speed,
        speed: speed
      };
    }
    /**
     * 设置漫游视角
     * @param {String} viewType 漫游视角（no~不固定视角/gs~跟随视角/dy~第一视角/sd~上帝视角）
     */

  }, {
    key: "setViewType",
    value: function setViewType(viewType, customXYZ) {
      this.viewType = viewType;

      switch (this.viewType) {
        case "dy":
          this.isLockView = true;
          this.viewXYZ = {
            x: 100,
            y: 0,
            z: 10
          };
          break;

        case "sd":
          this.isLockView = true;
          this.viewXYZ = {
            x: 0,
            y: 0,
            z: 5000
          };
          break;

        case "gs":
          this.isLockView = false;
          this.viewer.trackedEntity = this.roamEntity || undefined;
          break;

        case "custom":
          this.isLockView = true;
          customXYZ = customXYZ || {};
          this.viewXYZ = {
            x: customXYZ.x || 600,
            y: customXYZ.y || 0,
            z: customXYZ.z || 150
          };
          break;

        default:
          this.isLockView = false;
          this.viewer.trackedEntity = undefined;
          this.viewer.scene.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      }
    }
    /**
     * 设置自定义跟随视角
     * @param {Object} viewXYZ 视角参数
     * @param {Number} viewXYZ.x x方向偏移距离
     * @param {Number} viewXYZ.y y方向偏移距离
     * @param {Number} viewXYZ.x z方向偏移距离
     */

  }, {
    key: "setTrackView",
    value: function setTrackView(viewXYZ) {
      this.isLockView = true;
      this.viewXYZ = viewXYZ;
    }
    /**
     * 获取当前漫游的属性
     * @returns {Object} attr 当前漫游属性
     */

  }, {
    key: "getAttr",
    value: function getAttr() {
      var attr = {
        viewType: this.viewType,
        alldistance: this.alldistance,
        alltimes: this.alltimes,
        distanceED: this.distanceED,
        timeED: this.timeED,
        speed: this.speed,
        fixType: this.fixType,
        positions: this.reversePositions(this.positions, this.viewer),
        nowPosition: this.roamEntity ? this.roamEntity.position.getValue(this.viewer.clock.currentTime) : undefined
        /*entityType: this.opt.entityType,
                entityAttr: this.opt.entityAttr */

      };
      return Object.assign(this.opt, attr);
    }
  }]);

  return Roam;
}();

/**
 * 漫游控制类
 * @class
 * @description 漫游控制类，通过此类对象，可直接添加漫游对象，并对添加的漫游对象进行控制，而不用多次new Roam。
 */

var RoamTool = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础配置
   */
  function RoamTool(viewer, opt) {
    _classCallCheck(this, RoamTool);

    this.viewer = viewer;
    this.opt = opt || {};
    this.startRoamFun = null;
    this.endRoamFun = null;
    this.roamingFun = null;
    this.stopRoamFun = null;
    this.goonRoamFun = null;
    this.endCreateFun = null;
    /**
     * 漫游对象数组
     * @property {Array} roamList 漫游对象数组
     */

    this.roamList = [];
    /**
     * @property {Array} nowStartRoam 当前正在漫游对象
     */

    this.nowStartRoam = null;
  }
  /** 
   * 事件绑定
   * @param {String} type 事件类型（startRoam 开始漫游时 / endRoam 结束当前漫游时 / roaming 漫游过程中 / stopRoam 漫游暂停时 / goonRoam 继续漫游时 / endCreate 漫游路线绘制完成时）
   * @param {Function} fun 绑定函数
   */


  _createClass(RoamTool, [{
    key: "on",
    value: function on(type, fun) {
      if (type == "startRoam") {
        this.startRoamFun = fun;
      }

      if (type == "endRoam") {
        this.endRoamFun = fun;
      }

      if (type == "roaming") {
        this.roamingFun = fun;
      }

      if (type == "stopRoam") {
        this.stopRoamFun = fun;
      }

      if (type == "goonRoam") {
        this.goonRoamFun = fun;
      }

      if (type == "endCreate") {
        this.endCreateFun = fun;
      }
    }
    /**
     * 
     * @param {Object} opt 
     * @param {Array} [opt.positions] 漫游坐标数组
     * @param {Number} [opt.roamType=0] 漫游类型（1~飞行漫游/2~贴地漫游/0~普通漫游）
     * @param {Number} [opt.alltimes=60] 漫游时长，和speed互斥
     * @param {Number} [opt.speed] 漫游速度，和alltimes互斥
     * @param {String} [opt.viewType='no'] 漫游时视角类型
     * @param {Number} opt.height 坐标高度
     * @param {Function} callback 线路绘制完成后的回调，回调函数参数为当前创建的漫游对象
     */

  }, {
    key: "create",
    value: function create(opt, callback) {
      opt = opt || {};
      var _opt = opt,
          roamType = _opt.roamType,
          positions = _opt.positions;
      positions = this.transfromPositions(positions);
      var roam = null;
      var roamAttr = {
        alltimes: opt.alltimes,
        speed: opt.speed,
        endRoam: this.endRoamFun,
        roaming: this.roamingFun,
        viewType: opt.viewType
      };
      if (!opt.alltimes && !opt.speed) roamAttr.alltimes = 60; // 不设置速度和时长 默认以60s时长飞完

      roamAttr = Object.assign(opt, roamAttr);
      var that = this;

      switch (Number(roamType)) {
        case 1:
          // 飞行漫游
          if (!opt.height) {
            console.log("飞行漫游缺少高度！");
            return;
          }

          var newPositions = this.updatePositionsHeight(positions, opt.height);
          roamAttr.positions = newPositions;
          roam = new Roam(this.viewer, roamAttr);
          roam.attr = roamAttr;
          this.roamList.push(roam);
          if (callback) callback(roam);
          break;

        case 2:
          // 贴地漫游
          // this.getTerrainPositions(positions, function (newPositions) {
          //     roamAttr.positions = newPositions;
          //     roamAttr.heightReference = 1;
          //     roam = new Roam(that.viewer, roamAttr);
          //     roam.attr = roamAttr;
          //     that.roamList.push(roam);
          //     if (callback) callback(roam);
          // })
          // ============ 当前未实时计算高程 by 20230404 ============
          if (roamAttr.entityAttr) roamAttr.entityAttr.heightReference = 1;
          roam = new Roam(that.viewer, roamAttr);
          roam.attr = roamAttr;
          that.roamList.push(roam);
          if (callback) callback(roam);
          break;

        case 3:
          // 贴模型漫游
          break;

        default:
          // 默认是普通漫游
          roamAttr.positions = positions;
          roam = new Roam(this.viewer, roamAttr);
          roam.attr = roamAttr;
          this.roamList.push(roam);
          if (callback) callback(roam);
      }

      return roam;
    }
  }, {
    key: "transfromPositions",
    value: function transfromPositions(positions) {
      if (!positions) return;

      if (positions[0] instanceof Cesium.Cartesian3) {
        return positions;
      } else if (positions[0].x && positions[0].y && positions[0].z) {
        var arr = [];
        positions.forEach(function (item) {
          arr.push(new Cesium.Cartesian3(item.x, item.y, item.z));
        });
        return arr;
      } else {
        var newPositions = [];
        positions.forEach(function (element) {
          var p = Cesium.Cartesian3.fromDegrees(element[0], element[1], element[2] || 0);
          newPositions.push(p);
        });
        return newPositions;
      }
    }
  }, {
    key: "updatePositionsHeight",
    value: function updatePositionsHeight(positions, height) {
      if (height == undefined) return positions;
      if (!positions || positions.length < 2) return;
      var newPositions = [];
      positions.forEach(function (position) {
        var ctgc = Cesium.Cartographic.fromCartesian(position.clone());
        ctgc.height = Number(height);
        var p = Cesium.Cartographic.toCartesian(ctgc);
        newPositions.push(p);
      });
      return newPositions;
    } // 计算贴地高程

  }, {
    key: "getTerrainPositions",
    value: function getTerrainPositions(positions, callback) {
      if (!positions || positions.length < 2) return;
      var cgArr = [];

      for (var i = 0; i < positions.length; i++) {
        var cartesian = positions[i];
        var cg = Cesium.Cartographic.fromCartesian(cartesian);
        cgArr.push(cg);
      }

      var that = this;
      Cesium.sampleTerrainMostDetailed(this.viewer.terrainProvider, cgArr).then(function (updateLnglats) {
        var raisedPositions = that.viewer.scene.globe.ellipsoid.cartographicArrayToCartesianArray(updateLnglats); //转为世界坐标数组

        if (callback) callback(raisedPositions);
      });
    }
    /**
     * 根据构建时指定的属性获取当前漫游对象
     * @param {String} fieldName 字段名称
     * @param {String} fieldValue 字段值
     * @returns {Array} 漫游对象数组
     */

  }, {
    key: "getRoamByField",
    value: function getRoamByField(fieldName, fieldValue) {
      if (!fieldName) return [];
      var arr = [];

      for (var i = 0; i < this.roamList.length; i++) {
        var roam = this.roamList[i];

        if (roam.attr[fieldName] == fieldValue) {
          arr.push({
            roam: roam,
            index: i
          });
        }
      }

      return arr;
    }
    /**
     * 根据id移除漫游对象
     * @param {String | Number} roamId 漫游对象id
     */

  }, {
    key: "removeRoamById",
    value: function removeRoamById(roamId) {
      if (!roamId) return;

      for (var i = this.roamList.length - 1; i >= 0; i--) {
        var roam = this.roamList[i];

        if (roam.objId == roamId) {
          roam.destroy();
          this.roamList.splice(i, 1);
          break;
        }
      }
    }
    /**
     * 移除漫游对象
     * @param {Object} roam 漫游对象
     */

  }, {
    key: "removeRoam",
    value: function removeRoam(roam) {
      if (!roam) return;
      var roamId = roam.objId;
      this.removeRoamById(roamId);
    }
    /**
     * 开始漫游
     * @param {Object} roam 漫游对象
     */

  }, {
    key: "startRoam",
    value: function startRoam(roam) {
      this.endRoam();
      var roamId = roam.objId;

      for (var i = this.roamList.length - 1; i >= 0; i--) {
        var _roam = this.roamList[i];

        if (_roam.objId == roamId) {
          _roam.start();

          this.nowStartRoam = _roam;
          if (this.startRoamFun) this.startRoamFun(_roam);
          break;
        }
      }
    }
    /**
     * 结束当前漫游
     */

  }, {
    key: "endRoam",
    value: function endRoam() {
      if (this.nowStartRoam) {
        this.nowStartRoam.end();
        this.nowStartRoam = null;
      }
    }
    /**
     * 获取当前漫游的对象属性
     * @returns {Object} 漫游属性
     */

  }, {
    key: "getNowroamAttr",
    value: function getNowroamAttr() {
      if (!this.nowStartRoam) return {};
      var attr = Object.assign(this.nowStartRoam.attr, this.nowStartRoam.getAttr());
      return attr;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      for (var i = this.roamList.length - 1; i >= 0; i--) {
        var roam = this.roamList[i];
        roam.destroy();
      }

      this.roamList = [];
      this.startRoamFun = null;
      this.endRoamFun = null;
      this.roamingFun = null;
      this.stopRoamFun = null;
      this.goonRoamFun = null;
      this.endCreateFun = null;
    }
  }, {
    key: "removeAll",
    value: function removeAll() {
      for (var i = this.roamList.length - 1; i >= 0; i--) {
        var roam = this.roamList[i];
        roam.destroy();
      }

      this.roamList = [];
    }
    /**
     * 转化为json
     */

  }, {
    key: "toJson",
    value: function toJson() {
      var arr = [];

      for (var i = this.roamList.length - 1; i >= 0; i--) {
        var roam = this.roamList[i];
        arr.push(roam.getAttr());
      }

      return arr;
    }
  }]);

  return RoamTool;
}();

/**
 * @description 自定义圆图元
 * @class
 * @param {Cesium.Viewer} viewer
 * @param {Object} opt 
 * @param {Cesium.Color} opt.color 颜色
 * @param {Number} opt.radius 半径
 * @param {Number} opt.speed 速度
 */
var CustomCriclePrimitive = /*#__PURE__*/function () {
  function CustomCriclePrimitive(viewer, opt) {
    _classCallCheck(this, CustomCriclePrimitive);

    this.viewer = viewer;
    this.opt = opt || {};
    var defaultOpt = {
      radius: 1000,
      // 半径
      speed: 1.0,
      color: Cesium.Color.WHITE
    };
    this.opt = Object.assign(defaultOpt, this.opt);
    this.opt.color = this.opt.color instanceof Cesium.Color ? this.opt.color : Cesium.Color.fromCssColorString(this.opt.color);
    if (!this.opt.center) return;

    if (this.opt.center instanceof Cesium.Cartesian3) {
      this.position = this.opt.center;
    } else {
      this.position = Cesium.Cartesian3.fromDegrees(this.opt.center[0], this.opt.center[1], this.opt.center[2] || 0);
    }

    this._primitive = undefined;
    this.init();
    return this._primitive;
  }

  _createClass(CustomCriclePrimitive, [{
    key: "init",
    value: function init() {
      var rectanglePosition = this.getCoorsByRadius(this.position, this.opt.radius);
      var geometry = new Cesium.RectangleGeometry({
        rectangle: Cesium.Rectangle.fromCartesianArray(rectanglePosition),
        height: Cesium.Cartographic.fromCartesian(this.position.clone()).height
      });
      this._primitive = new Cesium.Primitive({
        geometryInstances: new Cesium.GeometryInstance({
          geometry: geometry
          /* modelMatrix : Cesium.Transforms.eastNorthUpToFixedFrame(center.clone()) */

        }),
        appearance: new Cesium.EllipsoidSurfaceAppearance({
          material: new Cesium.Material({
            fabric: {
              uniforms: {
                image: this.opt.image || this.getImage(),
                color: this.opt.color,
                speed: this.opt.speed || 1.0
              },
              source: "\n                            uniform float angle;\n                            uniform sampler2D image;\n                            uniform vec4 color;\n                            czm_material czm_getMaterial(czm_materialInput materialInput)\n                            {\n                                czm_material material = czm_getDefaultMaterial(materialInput);\n                                float angle = mod((czm_frameNumber/100.0) * speed ,360.0);\n                                vec2 st = materialInput.st;\n                                st.x = st.x - 0.5;\n                                st.y = st.y - 0.5;\n                                if(st.x * st.x + st.y * st.y <= 0.25){\n                                    float x = st.x * cos(angle) - st.y * sin(angle);\n                                    float y = st.y * cos(angle) + st.x * sin(angle);\n                                    st.x = x + 0.5;\n                                    st.y = y + 0.5;\n                                } else {\n                                    st.x = st.x + 0.5;\n                                    st.y = st.y + 0.5;\n                                }\n                                material.diffuse = czm_gammaCorrect(texture(image, st).rgb * color.rgb);\n                                material.alpha = texture(image, st).a * color.a;\n                                material.emission = vec3(0.2);\n                                return material;\n                            }"
            }
          })
        }),
        asynchronous: false
      });
    }
  }, {
    key: "getCoorsByRadius",
    value: function getCoorsByRadius(center, radius) {
      if (!center) return;
      var trans = Cesium.Transforms.eastNorthUpToFixedFrame(center.clone());
      var trans_inverse = Cesium.Matrix4.inverse(trans.clone(), new Cesium.Matrix4());
      var center_local = Cesium.Matrix4.multiplyByPoint(trans_inverse, center.clone(), new Cesium.Cartesian3());
      var y = new Cesium.Cartesian3(0, 1, 0);
      var newArr = [];

      for (var i = 45; i <= 360; i += 180) {
        var roate_mtx = Cesium.Matrix3.fromRotationZ(Cesium.Math.toRadians(i), new Cesium.Matrix3());
        var roate_c = Cesium.Matrix3.multiplyByVector(roate_mtx, y, new Cesium.Cartesian3());
        Cesium.Cartesian3.multiplyByScalar(roate_c, radius, roate_c);
        var new_c = Cesium.Cartesian3.add(center_local, roate_c, new Cesium.Cartesian3());
        var cart = Cesium.Matrix4.multiplyByPoint(trans, new_c.clone(), new Cesium.Cartesian3());
        newArr.push(cart);
      }

      return newArr;
    }
  }, {
    key: "getImage",
    value: function getImage() {
      var canvas = document.createElement("canvas");
      canvas.width = 512;
      canvas.height = 512;
      var context = canvas.getContext("2d");
      var rg = context.createRadialGradient(256, 256, 0, 256, 256, 256);
      rg.addColorStop(.1, "rgba(255, 255, 255, 1.0)");
      rg.addColorStop(.2, "rgba(255, 255, 255, 0.0)");
      rg.addColorStop(.3, "rgba(255, 255, 255, 0.9)");
      rg.addColorStop(.5, "rgba(255, 255, 255, 0.0)");
      rg.addColorStop(.9, "rgba(255, 255, 255, 0.2)");
      rg.addColorStop(1, "rgba(255, 255, 255, 1.0)");
      context.clearRect(0, 0, 512, 512);
      context.strokeStyle = "rgb(255, 255, 255)";
      context.setLineDash([80, 80]);
      context.lineWidth = 30;
      context.arc(256, 256, 180, 0, 2 * Math.PI, !0);
      context.stroke();
      context.beginPath();
      context.arc(256, 256, 256, 0, 2 * Math.PI, !0);
      context.fillStyle = rg;
      context.fill();
      context.restore();
      return canvas;
    }
  }]);

  return CustomCriclePrimitive;
}();

// 自定义多边形墙
var CustomWallPrimitive = /*#__PURE__*/function () {
  function CustomWallPrimitive(viewer, opt) {
    _classCallCheck(this, CustomWallPrimitive);

    _defineProperty(this, "getGeometryValue", function () {
      var e = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 1;
      var t = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : 1;
      var n = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 1;
      var a = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 30;
      var s = [e, 0],
          l = [t, 0],
          u = Cesium.Math.toRadians(360 / a),
          c = 1 / a;
      var h = [],
          m = [0, 1, a + 1, 1, a + 2, a + 1],
          d = [];
      h[0] = s[0], h[1] = s[1], h[2] = 0, h[3 * (a + 1)] = l[0], h[3 * (a + 1) + 1] = l[1], h[3 * (a + 1) + 2] = n, d[0] = 0, d[1] = 0, d[2 * (a + 1)] = 0;

      for (var _e = d[2 * (a + 1) + 1] = 1, _t, i, o, r; _e <= a; _e++) {
        _t = u * _e, i = c * _e, o = s[0] * Math.cos(_t) - s[1] * Math.sin(_t), r = s[1] * Math.cos(_t) + s[0] * Math.sin(_t), h[3 * _e] = o, h[3 * _e + 1] = r, o = l[h[3 * _e + 2] = 0] * Math.cos(_t) - l[1] * Math.sin(_t), r = l[1] * Math.cos(_t) + l[0] * Math.sin(_t), h[3 * (a + 1) + 3 * _e] = o, h[3 * (a + 1) + 3 * _e + 1] = r, h[3 * (a + 1) + 3 * _e + 2] = n, d[2 * _e] = i, d[2 * _e + 1] = 0, d[2 * (a + 1) + 2 * _e] = i, d[2 * (a + 1) + 2 * _e + 1] = 1, m.push(_e, _e + 1, a + _e + 1, _e + 1, a + _e + 2, a + _e + 1);
      }

      return {
        position: h,
        st: d,
        indices: m
      };
    });

    this.viewer = viewer;
    this.opt = opt || {};
    var defaultOpt = {
      radius: 600,
      // 半径
      height: 200,
      // 高度
      speed: 2,
      // 扩散事件
      number: 6,
      // 边数
      color: Cesium.Color.WHITE
    };
    this.opt = Object.assign(defaultOpt, this.opt);
    this.opt.color = this.opt.color instanceof Cesium.Color ? this.opt.color : Cesium.Color.fromCssColorString(this.opt.color);
    if (!this.opt.center) return;

    if (this.opt.center instanceof Cesium.Cartesian3) {
      this.position = this.opt.center;
    } else {
      this.position = Cesium.Cartesian3.fromDegrees(this.opt.center[0], this.opt.center[1], this.opt.center[2] || 0);
    }

    this.modelMatrix = Cesium.Transforms.eastNorthUpToFixedFrame(this.position);
    this._primitive = undefined;
    this.boundingSphere = new Cesium.BoundingSphere(new Cesium.Cartesian3(0, 0, 0), 1);
    this.primitiveType = this.opt.primitiveType || "TRANGLES";
    this.step = 0;
  }

  _createClass(CustomWallPrimitive, [{
    key: "update",
    value: function update(context, frameState, commandList) {
      if (this._primitive) {
        this._primitive.destroy();

        this._primitive = undefined;
      }

      var geoi = new Cesium.GeometryInstance({
        geometry: this.getGeometry(),
        modelMatrix: this.modelMatrix
      });
      this._primitive = new Cesium.Primitive({
        geometryInstances: geoi,
        appearance: this.getAppearance(),
        asynchronous: false
      });

      this._primitive.update(context, frameState, commandList);
    }
  }, {
    key: "destroy",
    value: function destroy() {
      if (this.primitive) this.viewer.scene.primitives.add(this.primitive);
    }
  }, {
    key: "getGeometry",
    value: function getGeometry() {
      this.opt.speed += .003 * 1.5;
      this.opt.speed = this.opt.speed > 1 ? 0 : this.opt.speed;
      var t = .5 * (1 - Math.cos(this.opt.speed * Math.PI * 2));
      var scale = [];
      scale[0] = scale[1] = this.opt.radius * (1 - Math.cos(this.opt.speed * Math.PI)) * .5, scale[2] = this.opt.height * t;
      var geometryValue = this.getGeometryValue(1, 1, 1, this.opt.number || 6);

      if (scale) {
        for (var e = 0; e < geometryValue.position.length - 2; e += 3) {
          geometryValue.position[e] *= scale[0];
          geometryValue.position[e + 1] *= scale[1];
          geometryValue.position[e + 2] *= scale[2];
        }
      }

      return new Cesium.Geometry({
        attributes: {
          position: new Cesium.GeometryAttribute({
            componentDatatype: Cesium.ComponentDatatype.DOUBLE,
            componentsPerAttribute: 3,
            values: geometryValue.position
          }),
          st: new Cesium.GeometryAttribute({
            componentDatatype: Cesium.ComponentDatatype.FLOAT,
            componentsPerAttribute: 2,
            values: geometryValue.st
          })
        },
        indices: geometryValue.indices,
        primitiveType: Cesium.PrimitiveType[this.primitiveType],
        boundingSphere: this.boundingSphere
      });
    }
  }, {
    key: "getAppearance",
    value: function getAppearance() {
      return new Cesium.EllipsoidSurfaceAppearance({
        material: new Cesium.Material({
          fabric: {
            uniforms: {
              color: this.opt.color
            },
            source: "\n                        uniform vec4 color;\n                        czm_material czm_getMaterial(czm_materialInput materialInput){\n                            czm_material material = czm_getDefaultMaterial(materialInput);\n                            vec2 st = materialInput.st;\n                            material.diffuse = czm_gammaCorrect(color.rgb);\n                            material.alpha = color.a * pow(1.0 - st.t, 2.0);\n                            material.emission = vec3(0.2);\n                            return material;\n                        }\n                    "
          }
        }),
        renderState: {
          cull: {
            enabled: !1
          }
        }
      });
    }
  }]);

  return CustomWallPrimitive;
}();

/**
 * @description 自定义锥体
 * @class
 * @param {Cesium.Viewer} viewer
 * @param {Object} opt 
 * @param {Cesium.Cartesian3} opt.center 中心点坐标
 * @param {Cesium.Color} opt.color 颜色
 * @param {Number} opt.radius 半径
 * @param {Number} opt.length 长度
 * @param {Number} opt.speed 速度
 */
var CustomCylinderPrimitive = /*#__PURE__*/function () {
  function CustomCylinderPrimitive(viewer, opt) {
    _classCallCheck(this, CustomCylinderPrimitive);

    _defineProperty(this, "getGeometryValue", function () {
      var e = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 1;
      var t = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : 1;
      var n = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 1;
      var a = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 30;
      var s = [e, 0],
          l = [t, 0],
          u = Cesium.Math.toRadians(360 / a),
          c = 1 / a;
      var h = [],
          m = [0, 1, a + 1, 1, a + 2, a + 1],
          d = [];
      h[0] = s[0], h[1] = s[1], h[2] = 0, h[3 * (a + 1)] = l[0], h[3 * (a + 1) + 1] = l[1], h[3 * (a + 1) + 2] = n, d[0] = 0, d[1] = 0, d[2 * (a + 1)] = 0;

      for (var _e = d[2 * (a + 1) + 1] = 1, _t, i, o, r; _e <= a; _e++) {
        _t = u * _e, i = c * _e, o = s[0] * Math.cos(_t) - s[1] * Math.sin(_t), r = s[1] * Math.cos(_t) + s[0] * Math.sin(_t), h[3 * _e] = o, h[3 * _e + 1] = r, o = l[h[3 * _e + 2] = 0] * Math.cos(_t) - l[1] * Math.sin(_t), r = l[1] * Math.cos(_t) + l[0] * Math.sin(_t), h[3 * (a + 1) + 3 * _e] = o, h[3 * (a + 1) + 3 * _e + 1] = r, h[3 * (a + 1) + 3 * _e + 2] = n, d[2 * _e] = i, d[2 * _e + 1] = 0, d[2 * (a + 1) + 2 * _e] = i, d[2 * (a + 1) + 2 * _e + 1] = 1, m.push(_e, _e + 1, a + _e + 1, _e + 1, a + _e + 2, a + _e + 1);
      }

      return {
        position: h,
        st: d,
        indices: m
      };
    });

    this.viewer = viewer;
    this.opt = opt || {};
    var defaultOpt = {
      radius: 10,
      // 半径
      length: 300,
      // 高度
      speed: 0,
      // 闪烁速度
      color: Cesium.Color.WHITE
    };
    this.opt = Object.assign(defaultOpt, this.opt);
    this.opt.color = this.opt.color instanceof Cesium.Color ? this.opt.color : Cesium.Color.fromCssColorString(this.opt.color);

    if (this.opt.center instanceof Cesium.Cartesian3) {
      this.position = this.opt.center.clone();
    } else {
      this.position = Cesium.Cartesian3.fromDegrees(this.opt.center[0], this.opt.center[1], this.opt.center[2] || 0);
    }

    this.modelMatrix = Cesium.Transforms.eastNorthUpToFixedFrame(this.position);
    this._primitive = undefined;
    this.boundingSphere = new Cesium.BoundingSphere(new Cesium.Cartesian3(0, 0, 0), 1);
    this.primitiveType = this.opt.primitiveType || "TRANGLES";
    var geoi = new Cesium.GeometryInstance({
      geometry: this.getGeometry(),
      modelMatrix: this.modelMatrix
    });
    this._primitive = new Cesium.Primitive({
      geometryInstances: geoi,
      appearance: this.opt.image ? this.getImgAppearance() : this.getAppearance(),
      asynchronous: false
    });
    return this._primitive;
  }
  /**
   * 销毁
   */


  _createClass(CustomCylinderPrimitive, [{
    key: "destroy",
    value: function destroy() {
      if (this.primitive) this.viewer.scene.primitives.add(this.primitive);
    }
  }, {
    key: "getGeometry",
    value: function getGeometry() {
      var scale = [this.opt.radius, this.opt.radius, this.opt.length];
      var geometryValue = this.getGeometryValue(2, .3, 1, 10);

      if (scale) {
        for (var e = 0; e < geometryValue.position.length - 2; e += 3) {
          geometryValue.position[e] *= scale[0];
          geometryValue.position[e + 1] *= scale[1];
          geometryValue.position[e + 2] *= scale[2];
        }
      }

      return new Cesium.Geometry({
        attributes: {
          position: new Cesium.GeometryAttribute({
            componentDatatype: Cesium.ComponentDatatype.DOUBLE,
            componentsPerAttribute: 3,
            values: geometryValue.position
          }),
          st: new Cesium.GeometryAttribute({
            componentDatatype: Cesium.ComponentDatatype.FLOAT,
            componentsPerAttribute: 2,
            values: geometryValue.st
          })
        },
        indices: geometryValue.indices,
        primitiveType: Cesium.PrimitiveType[this.opt.primitiveType],
        boundingSphere: this.boundingSphere
      });
    }
  }, {
    key: "getAppearance",
    value: function getAppearance() {
      return new Cesium.EllipsoidSurfaceAppearance({
        material: new Cesium.Material({
          fabric: {
            uniforms: {
              color: this.opt.color
            },
            source: "\n                        uniform vec4 color;\n                        czm_material czm_getMaterial(czm_materialInput materialInput){\n                            czm_material material = czm_getDefaultMaterial(materialInput);\n                            vec2 st = materialInput.st;\n                            float powerRatio = fract(czm_frameNumber / 30.0) + 1.0;\n                            float alpha = pow(1.0 - st.t, powerRatio);\n                            material.diffuse = czm_gammaCorrect(color.rgb);\n                            material.alpha = alpha * color.a;\n                            material.emission = vec3(0.2);\n                            return material;\n                        }\n                    "
          }
        }),
        renderState: {
          cull: {
            enabled: false
          }
        }
      });
    }
  }, {
    key: "getImgAppearance",
    value: function getImgAppearance() {
      return new Cesium.EllipsoidSurfaceAppearance({
        material: new Cesium.Material({
          fabric: {
            uniforms: {
              image: this.opt.image,
              color: Cesium.Color.WHITE
            },
            source: "\n                        uniform sampler2D image;\n                        uniform vec4 color;\n                        czm_material czm_getMaterial(czm_materialInput materialInput){\n                            czm_material material = czm_getDefaultMaterial(materialInput);\n                            vec2 v_st = materialInput.st;\n                            float dt = fract(czm_frameNumber / 90.0);\n                            vec2 st = fract(vec2(1.0) + v_st - vec2(dt, dt));\n                            vec4 imageColor = texture(image, st);\n                            vec3 diffuse = imageColor.rgb;\n                            float alpha = imageColor.a;\n                            diffuse *= color.rgb;\n                            alpha *= color.a;\n                            diffuse *= color.rgb;\n                            alpha *= color.a;\n                            material.diffuse = diffuse;\n                            material.alpha = alpha * pow(1.0 - v_st.t, 2.0);\n                            material.emission = vec3(0.2);\n                            return material;\n                        }\n                    "
          }
        }),
        renderState: {
          cull: {
            enabled: false
          }
        }
      });
    }
  }]);

  return CustomCylinderPrimitive;
}();

/**
 * @description 自定义锥体
 * @class
 * @param {Cesium.Viewer} viewer
 * @param {Object} opt 
 * @param {Cesium.Cartesian3} opt.position 顶点坐标
 * @param {Number} opt.distance 锥体长度
 * @param {Number} opt.fov 水平张角
 * @param {Number} opt.aspect 宽高比
 * @param {Cesium.Color} opt.color 颜色
 * @param {Number} opt.heading 偏转角
 * @param {Number} opt.pitch 仰俯角
 * @param {Number} opt.roll 翻滚角
 * @param {Cesium.Color} opt.outlineColor 边框颜色
 * @param {Cesium.Color} opt.rectMaterial 底部材质
 * @param {Boolean} opt.coneVisible 锥体是否显示 
 * @param {Boolean} opt.rectangleVisible 底部矩形是否显示 
 * @param {Number} opt.radius 半径
 * @param {Number} opt.speed 速度
 */
var ConePrimitive = /*#__PURE__*/function () {
  function ConePrimitive(viewer, opt) {
    _classCallCheck(this, ConePrimitive);

    this.viewer = viewer;
    opt = opt || {};
    this.position = opt.position; // 顶点坐标

    this.distance = opt.distance || 1000; // 长度

    this.direction = opt.direction || new Cesium.Cartesian3(1, 0, 0);
    this.fov = opt.fov || 60; // 水平张角

    this.aspect = opt.aspect || 2; // 宽高比

    /**
     * @property 锥体
     */

    this._primitive = undefined;
    this.heading = opt.heading || 0;
    this.pitch = opt.pitch || 0;
    this.roll = opt.roll || 0;
    this.positions = [];
    /**
     * @property 底部矩形
     */

    this.rectangle = undefined;
    this.outlineColor = opt.outlineColor || Cesium.Color.YELLOW.withAlpha(.5);
    this.rectMaterial = opt.rectMaterial || Cesium.Color.YELLOW;
    this.rectOnGround = opt.rectOnGround == undefined ? false : opt.rectOnGround;
    this.coneVisible = opt.coneVisible == undefined ? true : opt.coneVisible;
    this.rectangleVisible = opt.rectangleVisible;
    /*  this.polygonStyle = opt.polygon || {}; */

    this.stRotation = 0;
    this._visible = true;
  }

  _createClass(ConePrimitive, [{
    key: "getGeometry",
    value: function getGeometry(dis, fov, aspect, primitiveType) {
      var positions = new Float64Array(5 * 3);
      fov = Cesium.Math.toRadians(fov / 2);
      var tanfov = Math.tan(fov);
      var halfw = tanfov * dis;
      var halfh = halfw / aspect; // 如果想要heading pitch roll 正常，则建立正东方向的geometry
      // 点0 坐标

      positions[0] = 0.01;
      positions[1] = 0.01;
      positions[2] = 0.01; // 点1 坐标

      positions[3] = 1.0 * dis;
      positions[4] = 1.0 * halfw;
      positions[5] = 1.0 * halfh; // 点2 坐标

      positions[6] = 1.0 * dis;
      positions[7] = -1.0 * halfw;
      positions[8] = 1.0 * halfh; // 点3 坐标

      positions[9] = 1.0 * dis;
      positions[10] = -1.0 * halfw;
      positions[11] = -1.0 * halfh; // 点4 坐标

      positions[12] = 1.0 * dis;
      positions[13] = 1.0 * halfw;
      positions[14] = -1.0 * halfh; // 创建顶点属性中的坐标

      var attributes = new Cesium.GeometryAttributes({
        position: new Cesium.GeometryAttribute({
          componentDatatype: Cesium.ComponentDatatype.DOUBLE,
          componentsPerAttribute: 3,
          values: positions
        })
      }); // 点的索引

      var indices = new Uint16Array(18);
      indices[0] = 0;
      indices[1] = 1;
      indices[2] = 0;
      indices[3] = 2;
      indices[4] = 0;
      indices[5] = 3;
      indices[6] = 0;
      indices[7] = 4;
      indices[8] = 1;
      indices[9] = 2;
      indices[10] = 2;
      indices[11] = 3;
      indices[12] = 3;
      indices[13] = 4;
      indices[14] = 4;
      indices[15] = 1;
      var geometry = new Cesium.Geometry({
        attributes: attributes,
        indices: indices,
        primitiveType: Cesium.PrimitiveType.LINES,
        boundingSphere: Cesium.BoundingSphere.fromVertices(positions)
      });
      return geometry;
    }
  }, {
    key: "update",
    value: function update(context, frameState, commandList) {
      var geometry = this.getGeometry(this.distance, this.fov, this.aspect);

      if (!geometry) {
        return;
      }

      if (this._primitive) {
        this._primitive.destroy();

        this._primitive = undefined;
      }

      var headingPitchRoll = new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(this.heading), Cesium.Math.toRadians(this.pitch), Cesium.Math.toRadians(this.roll));
      var hprmtx = Cesium.Transforms.headingPitchRollToFixedFrame(this.position.clone(), headingPitchRoll);
      this._primitive = new Cesium.Primitive({
        geometryInstances: new Cesium.GeometryInstance({
          geometry: geometry,
          attributes: {
            color: Cesium.ColorGeometryInstanceAttribute.fromColor(this.outlineColor)
          }
        }),
        appearance: new Cesium.PerInstanceColorAppearance({
          translucent: true,
          flat: true
        }),
        modelMatrix: hprmtx,
        asynchronous: false,
        show: this.coneVisible
      });
      if (this.rectangleVisible) this.updateRectangle(hprmtx);

      this._primitive.update(context, frameState, commandList);
    }
  }, {
    key: "updateRectangle",
    value: function updateRectangle(hprmtx) {
      this.hierarchy = [];
      var angle = Cesium.Math.toRadians(this.fov / 2);
      var tanfov = Math.tan(angle);
      var halfw = tanfov * this.distance;
      var halfh = halfw / this.aspect;
      var modelMatrix_inverse = Cesium.Matrix4.inverse(hprmtx, new Cesium.Matrix4());
      var model_center = Cesium.Matrix4.multiplyByPoint(modelMatrix_inverse, this.position.clone(), new Cesium.Cartesian3());
      var newP1 = new Cesium.Cartesian3(model_center.x + this.distance, model_center.y + halfw, model_center.z + halfh);
      var pp1 = Cesium.Matrix4.multiplyByPoint(hprmtx, newP1.clone(), new Cesium.Cartesian3());
      var newP2 = new Cesium.Cartesian3(model_center.x + this.distance, model_center.y - halfw, model_center.z + halfh);
      var pp2 = Cesium.Matrix4.multiplyByPoint(hprmtx, newP2.clone(), new Cesium.Cartesian3());
      var newP3 = new Cesium.Cartesian3(model_center.x + this.distance, model_center.y - halfw, model_center.z - halfh);
      var pp3 = Cesium.Matrix4.multiplyByPoint(hprmtx, newP3.clone(), new Cesium.Cartesian3());
      var newP4 = new Cesium.Cartesian3(model_center.x + this.distance, model_center.y + halfw, model_center.z - halfh);
      var pp4 = Cesium.Matrix4.multiplyByPoint(hprmtx, newP4.clone(), new Cesium.Cartesian3());
      var that = this;
      that.positions = [pp4, pp3, pp2, pp1];

      if (!that.rectangle) {
        that.rectangle = that.viewer.entities.add({
          polygon: {
            hierarchy: new Cesium.CallbackProperty(function () {
              return new Cesium.PolygonHierarchy(that.positions);
            }, false),
            material: this.rectMaterial,
            perPositionHeight: !this.rectOnGround
          },
          show: this.rectangleVisible
        });
      }
      /* this.rectangle.polygon.stRotation = Cesium.Math.toRadians(
        this.stRotation || 90
      ); */

    }
  }, {
    key: "destroy",
    value: function destroy() {
      if (this.rectangle) {
        this.viewer.entities.remove(this.rectangle);
        this.rectangle = undefined;
      }
    }
    /**
     * 设置锥体顶点坐标
     * @param {Cesium.Cartesian3} position 
     */

  }, {
    key: "setPosition",
    value: function setPosition(position) {
      this.position = position;
    }
    /**
     * 设置锥体姿态
     * @param {Number} heading 偏转角
     * @param {Number} pitch 仰俯角
     * @param {Number} roll 翻滚角
     */

  }, {
    key: "setHeadingPitchRoll",
    value: function setHeadingPitchRoll(heading, pitch, roll) {
      if (heading != undefined) this.heading = heading;
      if (pitch != undefined) this.pitch = pitch;
      if (roll != undefined) this.roll = roll;
    }
    /**
     * 设置视频材质方向
     * @param {Number} angle 
     */

  }, {
    key: "setPolygonRotation",
    value: function setPolygonRotation(angle) {
      this.stRotation = angle || 0;
    }
  }, {
    key: "visible",
    get: function get() {
      return this.__visible;
    },
    set: function set(visible) {
      this.coneVisible = this.rectangleVisible = visible;
      this._visible = visible;
      if (this.rectangle) this.rectangle.show = visible;
    }
  }]);

  return ConePrimitive;
}();

/**
 * @param {Object} opt 参数
 * @param {Array | Cesium.Cartesian3} [opt.startPosition] 起点坐标
 * @param {Array | Cesium.endPosition} [opt.endPosition] 终点坐标
 * @param {Number} topRadius 顶部半径
 * @param {Number} bottomRadius 底部半径
 * @param {String | Cesium.Color} color 颜色
 */
var TrackCylinderGraphic = /*#__PURE__*/function () {
  function TrackCylinderGraphic(opt) {
    _classCallCheck(this, TrackCylinderGraphic);

    this.opt = opt || {};

    if (!this.opt.startPosition || !this.opt.endPosition) {
      console.log("缺少坐标信息");
      return;
    }

    this._startPosition = this.opt.startPosition instanceof Cesium.Cartesian3 ? this.opt.startPosition : Cesium.Cartesian3.fromDegrees(this._startPosition[0], this._startPosition[1], this._startPosition[2]);
    this._endPosition = this.opt.endPosition instanceof Cesium.Cartesian3 ? this.opt.endPosition : Cesium.Cartesian3.fromDegrees(this._endPosition[0], this._endPosition[1], this._endPosition[2]);
    var that = this;
    var color = this.opt.color || "#ffffff";

    if (!(color instanceof Cesium.Color)) {
      color = Cesium.Color.fromCssColorString(color);
    }

    this.entity = new Cesium.Entity({
      position: new Cesium.CallbackProperty(function () {
        return Cesium.Cartesian3.midpoint(that._startPosition.clone(), that._endPosition.clone(), new Cesium.Cartesian3());
      }, false),
      orientation: new Cesium.CallbackProperty(function () {
        var hpr = that.getHPR(that._startPosition, that._endPosition);
        return Cesium.Transforms.headingPitchRollQuaternion(that._startPosition, new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(hpr.heading), Cesium.Math.toRadians(hpr.pitch - 90), Cesium.Math.toRadians(hpr.roll)));
      }, false),
      cylinder: {
        length: new Cesium.CallbackProperty(function () {
          return Cesium.Cartesian3.distance(that._startPosition.clone(), that._endPosition.clone());
        }, false),
        topRadius: this.opt.topRadius || 200,
        bottomRadius: this.opt.bottomRadius || 0,
        outline: this.opt.outline,
        numberOfVerticalLines: this.opt.numberOfVerticalLines || 8,
        outlineColor: this.opt.outlineColor || Cesium.Color.BLACK.withAlpha(.7),
        outlineWidth: this.opt.outlineWidth || 1,
        material: color
      }
    });
    /*  return this.entity; */
  }

  _createClass(TrackCylinderGraphic, [{
    key: "startPosition",
    set: function set(p) {
      if (!p) return;
      this._startPosition = p.clone();
    }
  }, {
    key: "endPosition",
    set: function set(p) {
      if (!p) return;
      this._endPosition = p.clone();
    }
  }, {
    key: "getHPR",
    value: function getHPR(origin, end) {
      // 计算朝向
      var qc = Cesium.Cartesian3.subtract(end.clone(), origin.clone(), new Cesium.Cartesian3());
      qc = Cesium.Cartesian3.normalize(qc.clone(), new Cesium.Cartesian3()); // 计算向量旋转矩阵

      var rotationMatrix3 = Cesium.Transforms.rotationMatrixFromPositionVelocity(origin, qc, Cesium.Ellipsoid.WGS84); // 转4维矩阵

      var modelMatrix4 = Cesium.Matrix4.fromRotationTranslation(rotationMatrix3, origin); // 局部坐标矩阵

      var m1 = Cesium.Transforms.eastNorthUpToFixedFrame(origin, Cesium.Ellipsoid.WGS84, new Cesium.Matrix4());
      var m3 = Cesium.Matrix4.multiply(Cesium.Matrix4.inverse(m1, new Cesium.Matrix4()), modelMatrix4, new Cesium.Matrix4()); // 得到旋转矩阵

      var mat3 = Cesium.Matrix4.getMatrix3(m3, new Cesium.Matrix3()); // 计算四元数

      var q = Cesium.Quaternion.fromRotationMatrix(mat3);
      var hpr = Cesium.HeadingPitchRoll.fromQuaternion(q);
      var heading = Cesium.Math.toDegrees(hpr.heading);
      var pitch = Cesium.Math.toDegrees(hpr.pitch);
      var roll = Cesium.Math.toDegrees(hpr.roll);
      return {
        heading: heading,
        pitch: pitch,
        roll: roll
      };
    }
  }]);

  return TrackCylinderGraphic;
}();

/**
 * 雾场景
 * @property {Boolean} isActivate 是否激活
 * @property {Function} activate 开启场景
 * @property {Function} disable 关闭场景
 */
var fog = {
  fogProcs: null,

  /**
  * 是否激活
  */
  isActivate: false,

  /**
   * 能见度（0-1）
   */
  fogVal: 0.1,

  /**
   * 激活
   * @function
   */
  viewer: undefined,
  activate: function activate(viewer) {
    this.viewer = viewer || window.viewer;
    if (this.isActivate) return;
    this.isActivate = true;
    var fs_fog = this.initfog(); //整个场景通过后期渲染变亮 1为保持不变 大于1变亮 0-1变暗 uniforms后面为对应glsl里面定义的uniform参数
    // this.fogProcs.uniforms.brightness=2;

    this.fogProcs = new Cesium.PostProcessStage({
      name: 'czm_fog',
      fragmentShader: fs_fog
    });
    this.viewer.scene.postProcessStages.add(this.fogProcs);
  },

  /**
   * 销毁释放
   */
  disable: function disable() {
    if (!this.isActivate) return;
    this.isActivate = false;

    if (this.fogProcs) {
      this.viewer.scene.postProcessStages.remove(this.fogProcs);
      this.fogProcs.destroy();
      this.fogProcs = null;
    }
  },
  initfog: function initfog() {
    return "  uniform sampler2D colorTexture;\n" + "  uniform sampler2D depthTexture;\n" + "  in vec2 v_textureCoordinates;\n" + "  void main(void)\n" + "  {\n" + "      vec4 origcolor=texture(colorTexture, v_textureCoordinates);\n" + "      vec4 fogcolor=vec4(0.8,0.8,0.8,0.2);\n" + "\n" + "      vec4 depthcolor = texture(depthTexture, v_textureCoordinates);\n" + // 获取深度值
    "\n" + "      float f=(depthcolor.r-0.22)/" + (1 - this.fogVal) + ";\n" + "      if(f<0.0) f=0.0;\n" + "      else if(f>1.0) f=1.0;\n" + "      out_FragColor = mix(origcolor,fogcolor,f);\n" + "   }";
  }
};

/**
 * 下雪场景
 * @property {Boolean} isActivate 是否激活
 * @property {Function} activate 开启场景
 * @property {Function} disable 关闭场景
 */
var snow = {
  snowProcs: null,

  /**
   * 是否激活
   */
  isActivate: false,

  /**
   * 激活
   */
  activate: function activate(viewer) {
    this.viewer = viewer || window.viewer;
    if (this.isActivate) return;
    this.isActivate = true;
    var fs_snow = this.initSnow();
    this.snowProcs = new Cesium.PostProcessStage({
      name: 'czm_snow',
      fragmentShader: fs_snow
    });
    this.viewer.scene.postProcessStages.add(this.snowProcs);
  },

  /**
   * 销毁释放
   */
  disable: function disable() {
    if (!this.isActivate) return;
    this.isActivate = false;

    if (this.snowProcs) {
      this.viewer.scene.postProcessStages.remove(this.snowProcs);
      this.snowProcs = null;
    }
  },
  initSnow: function initSnow() {
    return "\n        uniform sampler2D colorTexture;\n        in vec2 v_textureCoordinates;\n        float snow(vec2 uv,float scale){\n            float time = czm_frameNumber / 60.0;\n            float w=smoothstep(1.,0.,-uv.y*(scale/10.));if(w<.1)return 0.;\n            uv+=time/scale;uv.y+=time*2./scale;uv.x+=sin(uv.y+time*.5)/scale;\n            uv*=scale;vec2 s=floor(uv),f=fract(uv),p;float k=3.,d;\n            p=.5+.35*sin(11.*fract(sin((s+p+scale)*mat2(7,3,6,5))*5.))-f;d=length(p);k=min(d,k);\n            k=smoothstep(0.,k,sin(f.x+f.y)*0.01);\n            return k*w;\n        }\n        \n        void main(void){\n            vec2 resolution = czm_viewport.zw;\n            vec2 uv=(gl_FragCoord.xy*2.-resolution.xy)/min(resolution.x,resolution.y);\n            vec3 finalColor=vec3(0);\n            float c = 0.0;\n            c+=snow(uv,30.)*.0;\n            c+=snow(uv,20.)*.0;\n            c+=snow(uv,15.)*.0;\n            c+=snow(uv,10.);\n            c+=snow(uv,8.);\n            c+=snow(uv,6.);\n            c+=snow(uv,5.);\n            finalColor=(vec3(c)); \n            out_FragColor = mix(texture(colorTexture, v_textureCoordinates), vec4(finalColor,1), 0.3); \n        }\n        ";
  }
};

/**
 * 下雨场景
 * @property {Boolean} isActivate 是否激活
 * @property {Function} activate 开启场景
 * @property {Function} disable 关闭场景
 */
var rain = {
  rainProcs: null,

  /** 
   * 是否开启
   * @property {Boolean} isActivate 是否开启
   * 
   */
  isActivate: false,

  /**
  * 激活
  */
  activate: function activate(viewer) {
    this.viewer = viewer || window.viewer;
    if (this.isActivate) return;
    this.isActivate = true;
    var fs_rain = this.initRain();
    this.rainProcs = new Cesium.PostProcessStage({
      name: 'czm_rain',
      fragmentShader: fs_rain
    });
    this.viewer.scene.postProcessStages.add(this.rainProcs);
  },

  /**
   * 销毁释放
   */
  disable: function disable() {
    if (!this.isActivate) return;
    this.isActivate = false;

    if (this.rainProcs) {
      this.viewer.scene.postProcessStages.remove(this.rainProcs);
      this.rainProcs = null;
    }
  },
  initRain: function initRain() {
    return "\n                uniform sampler2D colorTexture;\n                in vec2 v_textureCoordinates;\n                \n                float hash(float x){\n                    return fract(sin(x*23.3)*13.13);\n                }\n                \n                void main(void){\n                \n                    float time = czm_frameNumber / 60.0;\n                    // czm_viewport\u8868\u793A\u5F53\u524D\u7A97\u53E3\u7684\u5C3A\u5BF8\n                    vec2 resolution = czm_viewport.zw;\n                    // gl_FragCoord.xy\u8868\u793A\u5F53\u524D\u7A97\u53E3\u5750\u6807,\u5C06\u5F53\u524D\u5750\u6807\u6362\u7B97\u5230\u4EE5\u5C4F\u5E55\u4E2D\u5FC3\u70B9\u4E3A\u539F\u70B9\u7684\u5750\u6807\n                    vec2 uv=(gl_FragCoord.xy*2.-resolution.xy)/min(resolution.x,resolution.y);\n                    vec3 c=vec3(1.0,1.0,1.0); // \u8BBE\u7F6E\u96E8\u6C34\u989C\u8272\n                     \n                    float a = -.4;\n                    float si = sin(a);\n                    float co = cos(a);\n                    uv*=mat2(co,-si,si,co);\n                    uv*= 3.0;\n                    \n                    // \u6A2A\u5411\u5E73\u94FA\n                    float uvx = floor(uv.x*100.);\n                    float v=(1.-sin(hash(uvx)*2.))/2.0;\n\n                    float b=clamp(abs(sin(20.*time*v+uv.y*(5./(2.+v))))-.95,0.,1.)*20.;\n                    c*=v*b; \n                    out_FragColor = mix(texture(colorTexture, v_textureCoordinates), vec4(c,1), 0.3);  \n                }\n        ";
  }
};

/* eslint-disable no-unused-vars */

/* import MarkdownItSanitizer from 'markdown-it-sanitizer'
import MarkdownIt from 'markdown-it' */
var htmlTagRegex = /<html(.|\s)*>(.|\s)*<\/html>/im;
/* var md = new MarkdownIt({
  html: true,
  linkify: true
})

md.use(MarkdownItSanitizer, {
  imageClass: '',
  removeUnbalanced: false,
  removeUnknown: false
}) */

var KnockoutMarkdownBinding = {
  register: function register(Knockout) {
    Knockout.bindingHandlers.markdown = {
      'init': function init() {
        // Prevent binding on the dynamically-injected HTML (as developers are unlikely to expect that, and it has security implications)
        return {
          'controlsDescendantBindings': true
        };
      },
      'update': function update(element, valueAccessor) {
        // Remove existing children of this element.
        while (element.firstChild) {
          Knockout.removeNode(element.firstChild);
        }

        var rawText = Knockout.unwrap(valueAccessor()); // If the text contains an <html> tag, don't try to interpret it as Markdown because
        // we'll probably break it in the process.

        var html;

        if (htmlTagRegex.test(rawText)) {
          html = rawText;
        }

        var nodes = Knockout.utils.parseHtmlFragment(html, element);
        element.className = element.className + ' markdown';

        for (var i = 0; i < nodes.length; ++i) {
          var node = nodes[i];
          setAnchorTargets(node);
          element.appendChild(node);
        }
      }
    };
  }
};

function setAnchorTargets(element) {
  if (element instanceof HTMLAnchorElement) {
    element.target = '_blank';
  }

  if (element.childNodes && element.childNodes.length > 0) {
    for (var i = 0; i < element.childNodes.length; ++i) {
      setAnchorTargets(element.childNodes[i]);
    }
  }
}

/*! Hammer.JS - v2.0.7 - 2016-04-22
 * http://hammerjs.github.io/
 *
 * Copyright (c) 2016 Jorik Tangelder;
 * Licensed under the MIT license */
(function (window, document, exportName, undefined$1) {

  var VENDOR_PREFIXES = ['', 'webkit', 'Moz', 'MS', 'ms', 'o'];
  var TEST_ELEMENT = document.createElement('div');
  var TYPE_FUNCTION = 'function';
  var round = Math.round;
  var abs = Math.abs;
  var now = Date.now;
  /**
   * set a timeout with a given scope
   * @param {Function} fn
   * @param {Number} timeout
   * @param {Object} context
   * @returns {number}
   */

  function setTimeoutContext(fn, timeout, context) {
    return setTimeout(bindFn(fn, context), timeout);
  }
  /**
   * if the argument is an array, we want to execute the fn on each entry
   * if it aint an array we don't want to do a thing.
   * this is used by all the methods that accept a single and array argument.
   * @param {*|Array} arg
   * @param {String} fn
   * @param {Object} [context]
   * @returns {Boolean}
   */


  function invokeArrayArg(arg, fn, context) {
    if (Array.isArray(arg)) {
      each(arg, context[fn], context);
      return true;
    }

    return false;
  }
  /**
   * walk objects and arrays
   * @param {Object} obj
   * @param {Function} iterator
   * @param {Object} context
   */


  function each(obj, iterator, context) {
    var i;

    if (!obj) {
      return;
    }

    if (obj.forEach) {
      obj.forEach(iterator, context);
    } else if (obj.length !== undefined$1) {
      i = 0;

      while (i < obj.length) {
        iterator.call(context, obj[i], i, obj);
        i++;
      }
    } else {
      for (i in obj) {
        obj.hasOwnProperty(i) && iterator.call(context, obj[i], i, obj);
      }
    }
  }
  /**
   * wrap a method with a deprecation warning and stack trace
   * @param {Function} method
   * @param {String} name
   * @param {String} message
   * @returns {Function} A new function wrapping the supplied method.
   */


  function deprecate(method, name, message) {
    var deprecationMessage = 'DEPRECATED METHOD: ' + name + '\n' + message + ' AT \n';
    return function () {
      var e = new Error('get-stack-trace');
      var stack = e && e.stack ? e.stack.replace(/^[^\(]+?[\n$]/gm, '').replace(/^\s+at\s+/gm, '').replace(/^Object.<anonymous>\s*\(/gm, '{anonymous}()@') : 'Unknown Stack Trace';
      var log = window.console && (window.console.warn || window.console.log);

      if (log) {
        log.call(window.console, deprecationMessage, stack);
      }

      return method.apply(this, arguments);
    };
  }
  /**
   * extend object.
   * means that properties in dest will be overwritten by the ones in src.
   * @param {Object} target
   * @param {...Object} objects_to_assign
   * @returns {Object} target
   */


  var assign;

  if (typeof Object.assign !== 'function') {
    assign = function assign(target) {
      if (target === undefined$1 || target === null) {
        throw new TypeError('Cannot convert undefined or null to object');
      }

      var output = Object(target);

      for (var index = 1; index < arguments.length; index++) {
        var source = arguments[index];

        if (source !== undefined$1 && source !== null) {
          for (var nextKey in source) {
            if (source.hasOwnProperty(nextKey)) {
              output[nextKey] = source[nextKey];
            }
          }
        }
      }

      return output;
    };
  } else {
    assign = Object.assign;
  }
  /**
   * extend object.
   * means that properties in dest will be overwritten by the ones in src.
   * @param {Object} dest
   * @param {Object} src
   * @param {Boolean} [merge=false]
   * @returns {Object} dest
   */


  var extend = deprecate(function extend(dest, src, merge) {
    var keys = Object.keys(src);
    var i = 0;

    while (i < keys.length) {
      if (!merge || merge && dest[keys[i]] === undefined$1) {
        dest[keys[i]] = src[keys[i]];
      }

      i++;
    }

    return dest;
  }, 'extend', 'Use `assign`.');
  /**
   * merge the values from src in the dest.
   * means that properties that exist in dest will not be overwritten by src
   * @param {Object} dest
   * @param {Object} src
   * @returns {Object} dest
   */

  var merge = deprecate(function merge(dest, src) {
    return extend(dest, src, true);
  }, 'merge', 'Use `assign`.');
  /**
   * simple class inheritance
   * @param {Function} child
   * @param {Function} base
   * @param {Object} [properties]
   */

  function inherit(child, base, properties) {
    var baseP = base.prototype,
        childP;
    childP = child.prototype = Object.create(baseP);
    childP.constructor = child;
    childP._super = baseP;

    if (properties) {
      assign(childP, properties);
    }
  }
  /**
   * simple function bind
   * @param {Function} fn
   * @param {Object} context
   * @returns {Function}
   */


  function bindFn(fn, context) {
    return function boundFn() {
      return fn.apply(context, arguments);
    };
  }
  /**
   * let a boolean value also be a function that must return a boolean
   * this first item in args will be used as the context
   * @param {Boolean|Function} val
   * @param {Array} [args]
   * @returns {Boolean}
   */


  function boolOrFn(val, args) {
    if (_typeof(val) == TYPE_FUNCTION) {
      return val.apply(args ? args[0] || undefined$1 : undefined$1, args);
    }

    return val;
  }
  /**
   * use the val2 when val1 is undefined
   * @param {*} val1
   * @param {*} val2
   * @returns {*}
   */


  function ifUndefined(val1, val2) {
    return val1 === undefined$1 ? val2 : val1;
  }
  /**
   * addEventListener with multiple events at once
   * @param {EventTarget} target
   * @param {String} types
   * @param {Function} handler
   */


  function addEventListeners(target, types, handler) {
    each(splitStr(types), function (type) {
      target.addEventListener(type, handler, false);
    });
  }
  /**
   * removeEventListener with multiple events at once
   * @param {EventTarget} target
   * @param {String} types
   * @param {Function} handler
   */


  function removeEventListeners(target, types, handler) {
    each(splitStr(types), function (type) {
      target.removeEventListener(type, handler, false);
    });
  }
  /**
   * find if a node is in the given parent
   * @method hasParent
   * @param {HTMLElement} node
   * @param {HTMLElement} parent
   * @return {Boolean} found
   */


  function hasParent(node, parent) {
    while (node) {
      if (node == parent) {
        return true;
      }

      node = node.parentNode;
    }

    return false;
  }
  /**
   * small indexOf wrapper
   * @param {String} str
   * @param {String} find
   * @returns {Boolean} found
   */


  function inStr(str, find) {
    return str.indexOf(find) > -1;
  }
  /**
   * split string on whitespace
   * @param {String} str
   * @returns {Array} words
   */


  function splitStr(str) {
    return str.trim().split(/\s+/g);
  }
  /**
   * find if a array contains the object using indexOf or a simple polyFill
   * @param {Array} src
   * @param {String} find
   * @param {String} [findByKey]
   * @return {Boolean|Number} false when not found, or the index
   */


  function inArray(src, find, findByKey) {
    if (src.indexOf && !findByKey) {
      return src.indexOf(find);
    } else {
      var i = 0;

      while (i < src.length) {
        if (findByKey && src[i][findByKey] == find || !findByKey && src[i] === find) {
          return i;
        }

        i++;
      }

      return -1;
    }
  }
  /**
   * convert array-like objects to real arrays
   * @param {Object} obj
   * @returns {Array}
   */


  function toArray(obj) {
    return Array.prototype.slice.call(obj, 0);
  }
  /**
   * unique array with objects based on a key (like 'id') or just by the array's value
   * @param {Array} src [{id:1},{id:2},{id:1}]
   * @param {String} [key]
   * @param {Boolean} [sort=False]
   * @returns {Array} [{id:1},{id:2}]
   */


  function uniqueArray(src, key, sort) {
    var results = [];
    var values = [];
    var i = 0;

    while (i < src.length) {
      var val = key ? src[i][key] : src[i];

      if (inArray(values, val) < 0) {
        results.push(src[i]);
      }

      values[i] = val;
      i++;
    }

    if (sort) {
      if (!key) {
        results = results.sort();
      } else {
        results = results.sort(function sortUniqueArray(a, b) {
          return a[key] > b[key];
        });
      }
    }

    return results;
  }
  /**
   * get the prefixed property
   * @param {Object} obj
   * @param {String} property
   * @returns {String|Undefined} prefixed
   */


  function prefixed(obj, property) {
    var prefix, prop;
    var camelProp = property[0].toUpperCase() + property.slice(1);
    var i = 0;

    while (i < VENDOR_PREFIXES.length) {
      prefix = VENDOR_PREFIXES[i];
      prop = prefix ? prefix + camelProp : property;

      if (prop in obj) {
        return prop;
      }

      i++;
    }

    return undefined$1;
  }
  /**
   * get a unique id
   * @returns {number} uniqueId
   */


  var _uniqueId = 1;

  function uniqueId() {
    return _uniqueId++;
  }
  /**
   * get the window object of an element
   * @param {HTMLElement} element
   * @returns {DocumentView|Window}
   */


  function getWindowForElement(element) {
    var doc = element.ownerDocument || element;
    return doc.defaultView || doc.parentWindow || window;
  }

  var MOBILE_REGEX = /mobile|tablet|ip(ad|hone|od)|android/i;
  var SUPPORT_TOUCH = ('ontouchstart' in window);
  var SUPPORT_POINTER_EVENTS = prefixed(window, 'PointerEvent') !== undefined$1;
  var SUPPORT_ONLY_TOUCH = SUPPORT_TOUCH && MOBILE_REGEX.test(navigator.userAgent);
  var INPUT_TYPE_TOUCH = 'touch';
  var INPUT_TYPE_PEN = 'pen';
  var INPUT_TYPE_MOUSE = 'mouse';
  var INPUT_TYPE_KINECT = 'kinect';
  var COMPUTE_INTERVAL = 25;
  var INPUT_START = 1;
  var INPUT_MOVE = 2;
  var INPUT_END = 4;
  var INPUT_CANCEL = 8;
  var DIRECTION_NONE = 1;
  var DIRECTION_LEFT = 2;
  var DIRECTION_RIGHT = 4;
  var DIRECTION_UP = 8;
  var DIRECTION_DOWN = 16;
  var DIRECTION_HORIZONTAL = DIRECTION_LEFT | DIRECTION_RIGHT;
  var DIRECTION_VERTICAL = DIRECTION_UP | DIRECTION_DOWN;
  var DIRECTION_ALL = DIRECTION_HORIZONTAL | DIRECTION_VERTICAL;
  var PROPS_XY = ['x', 'y'];
  var PROPS_CLIENT_XY = ['clientX', 'clientY'];
  /**
   * create new input type manager
   * @param {Manager} manager
   * @param {Function} callback
   * @returns {Input}
   * @constructor
   */

  function Input(manager, callback) {
    var self = this;
    this.manager = manager;
    this.callback = callback;
    this.element = manager.element;
    this.target = manager.options.inputTarget; // smaller wrapper around the handler, for the scope and the enabled state of the manager,
    // so when disabled the input events are completely bypassed.

    this.domHandler = function (ev) {
      if (boolOrFn(manager.options.enable, [manager])) {
        self.handler(ev);
      }
    };

    this.init();
  }

  Input.prototype = {
    /**
     * should handle the inputEvent data and trigger the callback
     * @virtual
     */
    handler: function handler() {},

    /**
     * bind the events
     */
    init: function init() {
      this.evEl && addEventListeners(this.element, this.evEl, this.domHandler);
      this.evTarget && addEventListeners(this.target, this.evTarget, this.domHandler);
      this.evWin && addEventListeners(getWindowForElement(this.element), this.evWin, this.domHandler);
    },

    /**
     * unbind the events
     */
    destroy: function destroy() {
      this.evEl && removeEventListeners(this.element, this.evEl, this.domHandler);
      this.evTarget && removeEventListeners(this.target, this.evTarget, this.domHandler);
      this.evWin && removeEventListeners(getWindowForElement(this.element), this.evWin, this.domHandler);
    }
  };
  /**
   * create new input type manager
   * called by the Manager constructor
   * @param {Hammer} manager
   * @returns {Input}
   */

  function createInputInstance(manager) {
    var Type;
    var inputClass = manager.options.inputClass;

    if (inputClass) {
      Type = inputClass;
    } else if (SUPPORT_POINTER_EVENTS) {
      Type = PointerEventInput;
    } else if (SUPPORT_ONLY_TOUCH) {
      Type = TouchInput;
    } else if (!SUPPORT_TOUCH) {
      Type = MouseInput;
    } else {
      Type = TouchMouseInput;
    }

    return new Type(manager, inputHandler);
  }
  /**
   * handle input events
   * @param {Manager} manager
   * @param {String} eventType
   * @param {Object} input
   */


  function inputHandler(manager, eventType, input) {
    var pointersLen = input.pointers.length;
    var changedPointersLen = input.changedPointers.length;
    var isFirst = eventType & INPUT_START && pointersLen - changedPointersLen === 0;
    var isFinal = eventType & (INPUT_END | INPUT_CANCEL) && pointersLen - changedPointersLen === 0;
    input.isFirst = !!isFirst;
    input.isFinal = !!isFinal;

    if (isFirst) {
      manager.session = {};
    } // source event is the normalized value of the domEvents
    // like 'touchstart, mouseup, pointerdown'


    input.eventType = eventType; // compute scale, rotation etc

    computeInputData(manager, input); // emit secret event

    manager.emit('hammer.input', input);
    manager.recognize(input);
    manager.session.prevInput = input;
  }
  /**
   * extend the data with some usable properties like scale, rotate, velocity etc
   * @param {Object} manager
   * @param {Object} input
   */


  function computeInputData(manager, input) {
    var session = manager.session;
    var pointers = input.pointers;
    var pointersLength = pointers.length; // store the first input to calculate the distance and direction

    if (!session.firstInput) {
      session.firstInput = simpleCloneInputData(input);
    } // to compute scale and rotation we need to store the multiple touches


    if (pointersLength > 1 && !session.firstMultiple) {
      session.firstMultiple = simpleCloneInputData(input);
    } else if (pointersLength === 1) {
      session.firstMultiple = false;
    }

    var firstInput = session.firstInput;
    var firstMultiple = session.firstMultiple;
    var offsetCenter = firstMultiple ? firstMultiple.center : firstInput.center;
    var center = input.center = getCenter(pointers);
    input.timeStamp = now();
    input.deltaTime = input.timeStamp - firstInput.timeStamp;
    input.angle = getAngle(offsetCenter, center);
    input.distance = getDistance(offsetCenter, center);
    computeDeltaXY(session, input);
    input.offsetDirection = getDirection(input.deltaX, input.deltaY);
    var overallVelocity = getVelocity(input.deltaTime, input.deltaX, input.deltaY);
    input.overallVelocityX = overallVelocity.x;
    input.overallVelocityY = overallVelocity.y;
    input.overallVelocity = abs(overallVelocity.x) > abs(overallVelocity.y) ? overallVelocity.x : overallVelocity.y;
    input.scale = firstMultiple ? getScale(firstMultiple.pointers, pointers) : 1;
    input.rotation = firstMultiple ? getRotation(firstMultiple.pointers, pointers) : 0;
    input.maxPointers = !session.prevInput ? input.pointers.length : input.pointers.length > session.prevInput.maxPointers ? input.pointers.length : session.prevInput.maxPointers;
    computeIntervalInputData(session, input); // find the correct target

    var target = manager.element;

    if (hasParent(input.srcEvent.target, target)) {
      target = input.srcEvent.target;
    }

    input.target = target;
  }

  function computeDeltaXY(session, input) {
    var center = input.center;
    var offset = session.offsetDelta || {};
    var prevDelta = session.prevDelta || {};
    var prevInput = session.prevInput || {};

    if (input.eventType === INPUT_START || prevInput.eventType === INPUT_END) {
      prevDelta = session.prevDelta = {
        x: prevInput.deltaX || 0,
        y: prevInput.deltaY || 0
      };
      offset = session.offsetDelta = {
        x: center.x,
        y: center.y
      };
    }

    input.deltaX = prevDelta.x + (center.x - offset.x);
    input.deltaY = prevDelta.y + (center.y - offset.y);
  }
  /**
   * velocity is calculated every x ms
   * @param {Object} session
   * @param {Object} input
   */


  function computeIntervalInputData(session, input) {
    var last = session.lastInterval || input,
        deltaTime = input.timeStamp - last.timeStamp,
        velocity,
        velocityX,
        velocityY,
        direction;

    if (input.eventType != INPUT_CANCEL && (deltaTime > COMPUTE_INTERVAL || last.velocity === undefined$1)) {
      var deltaX = input.deltaX - last.deltaX;
      var deltaY = input.deltaY - last.deltaY;
      var v = getVelocity(deltaTime, deltaX, deltaY);
      velocityX = v.x;
      velocityY = v.y;
      velocity = abs(v.x) > abs(v.y) ? v.x : v.y;
      direction = getDirection(deltaX, deltaY);
      session.lastInterval = input;
    } else {
      // use latest velocity info if it doesn't overtake a minimum period
      velocity = last.velocity;
      velocityX = last.velocityX;
      velocityY = last.velocityY;
      direction = last.direction;
    }

    input.velocity = velocity;
    input.velocityX = velocityX;
    input.velocityY = velocityY;
    input.direction = direction;
  }
  /**
   * create a simple clone from the input used for storage of firstInput and firstMultiple
   * @param {Object} input
   * @returns {Object} clonedInputData
   */


  function simpleCloneInputData(input) {
    // make a simple copy of the pointers because we will get a reference if we don't
    // we only need clientXY for the calculations
    var pointers = [];
    var i = 0;

    while (i < input.pointers.length) {
      pointers[i] = {
        clientX: round(input.pointers[i].clientX),
        clientY: round(input.pointers[i].clientY)
      };
      i++;
    }

    return {
      timeStamp: now(),
      pointers: pointers,
      center: getCenter(pointers),
      deltaX: input.deltaX,
      deltaY: input.deltaY
    };
  }
  /**
   * get the center of all the pointers
   * @param {Array} pointers
   * @return {Object} center contains `x` and `y` properties
   */


  function getCenter(pointers) {
    var pointersLength = pointers.length; // no need to loop when only one touch

    if (pointersLength === 1) {
      return {
        x: round(pointers[0].clientX),
        y: round(pointers[0].clientY)
      };
    }

    var x = 0,
        y = 0,
        i = 0;

    while (i < pointersLength) {
      x += pointers[i].clientX;
      y += pointers[i].clientY;
      i++;
    }

    return {
      x: round(x / pointersLength),
      y: round(y / pointersLength)
    };
  }
  /**
   * calculate the velocity between two points. unit is in px per ms.
   * @param {Number} deltaTime
   * @param {Number} x
   * @param {Number} y
   * @return {Object} velocity `x` and `y`
   */


  function getVelocity(deltaTime, x, y) {
    return {
      x: x / deltaTime || 0,
      y: y / deltaTime || 0
    };
  }
  /**
   * get the direction between two points
   * @param {Number} x
   * @param {Number} y
   * @return {Number} direction
   */


  function getDirection(x, y) {
    if (x === y) {
      return DIRECTION_NONE;
    }

    if (abs(x) >= abs(y)) {
      return x < 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
    }

    return y < 0 ? DIRECTION_UP : DIRECTION_DOWN;
  }
  /**
   * calculate the absolute distance between two points
   * @param {Object} p1 {x, y}
   * @param {Object} p2 {x, y}
   * @param {Array} [props] containing x and y keys
   * @return {Number} distance
   */


  function getDistance(p1, p2, props) {
    if (!props) {
      props = PROPS_XY;
    }

    var x = p2[props[0]] - p1[props[0]],
        y = p2[props[1]] - p1[props[1]];
    return Math.sqrt(x * x + y * y);
  }
  /**
   * calculate the angle between two coordinates
   * @param {Object} p1
   * @param {Object} p2
   * @param {Array} [props] containing x and y keys
   * @return {Number} angle
   */


  function getAngle(p1, p2, props) {
    if (!props) {
      props = PROPS_XY;
    }

    var x = p2[props[0]] - p1[props[0]],
        y = p2[props[1]] - p1[props[1]];
    return Math.atan2(y, x) * 180 / Math.PI;
  }
  /**
   * calculate the rotation degrees between two pointersets
   * @param {Array} start array of pointers
   * @param {Array} end array of pointers
   * @return {Number} rotation
   */


  function getRotation(start, end) {
    return getAngle(end[1], end[0], PROPS_CLIENT_XY) + getAngle(start[1], start[0], PROPS_CLIENT_XY);
  }
  /**
   * calculate the scale factor between two pointersets
   * no scale is 1, and goes down to 0 when pinched together, and bigger when pinched out
   * @param {Array} start array of pointers
   * @param {Array} end array of pointers
   * @return {Number} scale
   */


  function getScale(start, end) {
    return getDistance(end[0], end[1], PROPS_CLIENT_XY) / getDistance(start[0], start[1], PROPS_CLIENT_XY);
  }

  var MOUSE_INPUT_MAP = {
    mousedown: INPUT_START,
    mousemove: INPUT_MOVE,
    mouseup: INPUT_END
  };
  var MOUSE_ELEMENT_EVENTS = 'mousedown';
  var MOUSE_WINDOW_EVENTS = 'mousemove mouseup';
  /**
   * Mouse events input
   * @constructor
   * @extends Input
   */

  function MouseInput() {
    this.evEl = MOUSE_ELEMENT_EVENTS;
    this.evWin = MOUSE_WINDOW_EVENTS;
    this.pressed = false; // mousedown state

    Input.apply(this, arguments);
  }

  inherit(MouseInput, Input, {
    /**
     * handle mouse events
     * @param {Object} ev
     */
    handler: function MEhandler(ev) {
      var eventType = MOUSE_INPUT_MAP[ev.type]; // on start we want to have the left mouse button down

      if (eventType & INPUT_START && ev.button === 0) {
        this.pressed = true;
      }

      if (eventType & INPUT_MOVE && ev.which !== 1) {
        eventType = INPUT_END;
      } // mouse must be down


      if (!this.pressed) {
        return;
      }

      if (eventType & INPUT_END) {
        this.pressed = false;
      }

      this.callback(this.manager, eventType, {
        pointers: [ev],
        changedPointers: [ev],
        pointerType: INPUT_TYPE_MOUSE,
        srcEvent: ev
      });
    }
  });
  var POINTER_INPUT_MAP = {
    pointerdown: INPUT_START,
    pointermove: INPUT_MOVE,
    pointerup: INPUT_END,
    pointercancel: INPUT_CANCEL,
    pointerout: INPUT_CANCEL
  }; // in IE10 the pointer types is defined as an enum

  var IE10_POINTER_TYPE_ENUM = {
    2: INPUT_TYPE_TOUCH,
    3: INPUT_TYPE_PEN,
    4: INPUT_TYPE_MOUSE,
    5: INPUT_TYPE_KINECT // see https://twitter.com/jacobrossi/status/480596438489890816

  };
  var POINTER_ELEMENT_EVENTS = 'pointerdown';
  var POINTER_WINDOW_EVENTS = 'pointermove pointerup pointercancel'; // IE10 has prefixed support, and case-sensitive

  if (window.MSPointerEvent && !window.PointerEvent) {
    POINTER_ELEMENT_EVENTS = 'MSPointerDown';
    POINTER_WINDOW_EVENTS = 'MSPointerMove MSPointerUp MSPointerCancel';
  }
  /**
   * Pointer events input
   * @constructor
   * @extends Input
   */


  function PointerEventInput() {
    this.evEl = POINTER_ELEMENT_EVENTS;
    this.evWin = POINTER_WINDOW_EVENTS;
    Input.apply(this, arguments);
    this.store = this.manager.session.pointerEvents = [];
  }

  inherit(PointerEventInput, Input, {
    /**
     * handle mouse events
     * @param {Object} ev
     */
    handler: function PEhandler(ev) {
      var store = this.store;
      var removePointer = false;
      var eventTypeNormalized = ev.type.toLowerCase().replace('ms', '');
      var eventType = POINTER_INPUT_MAP[eventTypeNormalized];
      var pointerType = IE10_POINTER_TYPE_ENUM[ev.pointerType] || ev.pointerType;
      var isTouch = pointerType == INPUT_TYPE_TOUCH; // get index of the event in the store

      var storeIndex = inArray(store, ev.pointerId, 'pointerId'); // start and mouse must be down

      if (eventType & INPUT_START && (ev.button === 0 || isTouch)) {
        if (storeIndex < 0) {
          store.push(ev);
          storeIndex = store.length - 1;
        }
      } else if (eventType & (INPUT_END | INPUT_CANCEL)) {
        removePointer = true;
      } // it not found, so the pointer hasn't been down (so it's probably a hover)


      if (storeIndex < 0) {
        return;
      } // update the event in the store


      store[storeIndex] = ev;
      this.callback(this.manager, eventType, {
        pointers: store,
        changedPointers: [ev],
        pointerType: pointerType,
        srcEvent: ev
      });

      if (removePointer) {
        // remove from the store
        store.splice(storeIndex, 1);
      }
    }
  });
  var SINGLE_TOUCH_INPUT_MAP = {
    touchstart: INPUT_START,
    touchmove: INPUT_MOVE,
    touchend: INPUT_END,
    touchcancel: INPUT_CANCEL
  };
  var SINGLE_TOUCH_TARGET_EVENTS = 'touchstart';
  var SINGLE_TOUCH_WINDOW_EVENTS = 'touchstart touchmove touchend touchcancel';
  /**
   * Touch events input
   * @constructor
   * @extends Input
   */

  function SingleTouchInput() {
    this.evTarget = SINGLE_TOUCH_TARGET_EVENTS;
    this.evWin = SINGLE_TOUCH_WINDOW_EVENTS;
    this.started = false;
    Input.apply(this, arguments);
  }

  inherit(SingleTouchInput, Input, {
    handler: function TEhandler(ev) {
      var type = SINGLE_TOUCH_INPUT_MAP[ev.type]; // should we handle the touch events?

      if (type === INPUT_START) {
        this.started = true;
      }

      if (!this.started) {
        return;
      }

      var touches = normalizeSingleTouches.call(this, ev, type); // when done, reset the started state

      if (type & (INPUT_END | INPUT_CANCEL) && touches[0].length - touches[1].length === 0) {
        this.started = false;
      }

      this.callback(this.manager, type, {
        pointers: touches[0],
        changedPointers: touches[1],
        pointerType: INPUT_TYPE_TOUCH,
        srcEvent: ev
      });
    }
  });
  /**
   * @this {TouchInput}
   * @param {Object} ev
   * @param {Number} type flag
   * @returns {undefined|Array} [all, changed]
   */

  function normalizeSingleTouches(ev, type) {
    var all = toArray(ev.touches);
    var changed = toArray(ev.changedTouches);

    if (type & (INPUT_END | INPUT_CANCEL)) {
      all = uniqueArray(all.concat(changed), 'identifier', true);
    }

    return [all, changed];
  }

  var TOUCH_INPUT_MAP = {
    touchstart: INPUT_START,
    touchmove: INPUT_MOVE,
    touchend: INPUT_END,
    touchcancel: INPUT_CANCEL
  };
  var TOUCH_TARGET_EVENTS = 'touchstart touchmove touchend touchcancel';
  /**
   * Multi-user touch events input
   * @constructor
   * @extends Input
   */

  function TouchInput() {
    this.evTarget = TOUCH_TARGET_EVENTS;
    this.targetIds = {};
    Input.apply(this, arguments);
  }

  inherit(TouchInput, Input, {
    handler: function MTEhandler(ev) {
      var type = TOUCH_INPUT_MAP[ev.type];
      var touches = getTouches.call(this, ev, type);

      if (!touches) {
        return;
      }

      this.callback(this.manager, type, {
        pointers: touches[0],
        changedPointers: touches[1],
        pointerType: INPUT_TYPE_TOUCH,
        srcEvent: ev
      });
    }
  });
  /**
   * @this {TouchInput}
   * @param {Object} ev
   * @param {Number} type flag
   * @returns {undefined|Array} [all, changed]
   */

  function getTouches(ev, type) {
    var allTouches = toArray(ev.touches);
    var targetIds = this.targetIds; // when there is only one touch, the process can be simplified

    if (type & (INPUT_START | INPUT_MOVE) && allTouches.length === 1) {
      targetIds[allTouches[0].identifier] = true;
      return [allTouches, allTouches];
    }

    var i,
        targetTouches,
        changedTouches = toArray(ev.changedTouches),
        changedTargetTouches = [],
        target = this.target; // get target touches from touches

    targetTouches = allTouches.filter(function (touch) {
      return hasParent(touch.target, target);
    }); // collect touches

    if (type === INPUT_START) {
      i = 0;

      while (i < targetTouches.length) {
        targetIds[targetTouches[i].identifier] = true;
        i++;
      }
    } // filter changed touches to only contain touches that exist in the collected target ids


    i = 0;

    while (i < changedTouches.length) {
      if (targetIds[changedTouches[i].identifier]) {
        changedTargetTouches.push(changedTouches[i]);
      } // cleanup removed touches


      if (type & (INPUT_END | INPUT_CANCEL)) {
        delete targetIds[changedTouches[i].identifier];
      }

      i++;
    }

    if (!changedTargetTouches.length) {
      return;
    }

    return [// merge targetTouches with changedTargetTouches so it contains ALL touches, including 'end' and 'cancel'
    uniqueArray(targetTouches.concat(changedTargetTouches), 'identifier', true), changedTargetTouches];
  }
  /**
   * Combined touch and mouse input
   *
   * Touch has a higher priority then mouse, and while touching no mouse events are allowed.
   * This because touch devices also emit mouse events while doing a touch.
   *
   * @constructor
   * @extends Input
   */


  var DEDUP_TIMEOUT = 2500;
  var DEDUP_DISTANCE = 25;

  function TouchMouseInput() {
    Input.apply(this, arguments);
    var handler = bindFn(this.handler, this);
    this.touch = new TouchInput(this.manager, handler);
    this.mouse = new MouseInput(this.manager, handler);
    this.primaryTouch = null;
    this.lastTouches = [];
  }

  inherit(TouchMouseInput, Input, {
    /**
     * handle mouse and touch events
     * @param {Hammer} manager
     * @param {String} inputEvent
     * @param {Object} inputData
     */
    handler: function TMEhandler(manager, inputEvent, inputData) {
      var isTouch = inputData.pointerType == INPUT_TYPE_TOUCH,
          isMouse = inputData.pointerType == INPUT_TYPE_MOUSE;

      if (isMouse && inputData.sourceCapabilities && inputData.sourceCapabilities.firesTouchEvents) {
        return;
      } // when we're in a touch event, record touches to  de-dupe synthetic mouse event


      if (isTouch) {
        recordTouches.call(this, inputEvent, inputData);
      } else if (isMouse && isSyntheticEvent.call(this, inputData)) {
        return;
      }

      this.callback(manager, inputEvent, inputData);
    },

    /**
     * remove the event listeners
     */
    destroy: function destroy() {
      this.touch.destroy();
      this.mouse.destroy();
    }
  });

  function recordTouches(eventType, eventData) {
    if (eventType & INPUT_START) {
      this.primaryTouch = eventData.changedPointers[0].identifier;
      setLastTouch.call(this, eventData);
    } else if (eventType & (INPUT_END | INPUT_CANCEL)) {
      setLastTouch.call(this, eventData);
    }
  }

  function setLastTouch(eventData) {
    var touch = eventData.changedPointers[0];

    if (touch.identifier === this.primaryTouch) {
      var lastTouch = {
        x: touch.clientX,
        y: touch.clientY
      };
      this.lastTouches.push(lastTouch);
      var lts = this.lastTouches;

      var removeLastTouch = function removeLastTouch() {
        var i = lts.indexOf(lastTouch);

        if (i > -1) {
          lts.splice(i, 1);
        }
      };

      setTimeout(removeLastTouch, DEDUP_TIMEOUT);
    }
  }

  function isSyntheticEvent(eventData) {
    var x = eventData.srcEvent.clientX,
        y = eventData.srcEvent.clientY;

    for (var i = 0; i < this.lastTouches.length; i++) {
      var t = this.lastTouches[i];
      var dx = Math.abs(x - t.x),
          dy = Math.abs(y - t.y);

      if (dx <= DEDUP_DISTANCE && dy <= DEDUP_DISTANCE) {
        return true;
      }
    }

    return false;
  }

  var PREFIXED_TOUCH_ACTION = prefixed(TEST_ELEMENT.style, 'touchAction');
  var NATIVE_TOUCH_ACTION = PREFIXED_TOUCH_ACTION !== undefined$1; // magical touchAction value

  var TOUCH_ACTION_COMPUTE = 'compute';
  var TOUCH_ACTION_AUTO = 'auto';
  var TOUCH_ACTION_MANIPULATION = 'manipulation'; // not implemented

  var TOUCH_ACTION_NONE = 'none';
  var TOUCH_ACTION_PAN_X = 'pan-x';
  var TOUCH_ACTION_PAN_Y = 'pan-y';
  var TOUCH_ACTION_MAP = getTouchActionProps();
  /**
   * Touch Action
   * sets the touchAction property or uses the js alternative
   * @param {Manager} manager
   * @param {String} value
   * @constructor
   */

  function TouchAction(manager, value) {
    this.manager = manager;
    this.set(value);
  }

  TouchAction.prototype = {
    /**
     * set the touchAction value on the element or enable the polyfill
     * @param {String} value
     */
    set: function set(value) {
      // find out the touch-action by the event handlers
      if (value == TOUCH_ACTION_COMPUTE) {
        value = this.compute();
      }

      if (NATIVE_TOUCH_ACTION && this.manager.element.style && TOUCH_ACTION_MAP[value]) {
        this.manager.element.style[PREFIXED_TOUCH_ACTION] = value;
      }

      this.actions = value.toLowerCase().trim();
    },

    /**
     * just re-set the touchAction value
     */
    update: function update() {
      this.set(this.manager.options.touchAction);
    },

    /**
     * compute the value for the touchAction property based on the recognizer's settings
     * @returns {String} value
     */
    compute: function compute() {
      var actions = [];
      each(this.manager.recognizers, function (recognizer) {
        if (boolOrFn(recognizer.options.enable, [recognizer])) {
          actions = actions.concat(recognizer.getTouchAction());
        }
      });
      return cleanTouchActions(actions.join(' '));
    },

    /**
     * this method is called on each input cycle and provides the preventing of the browser behavior
     * @param {Object} input
     */
    preventDefaults: function preventDefaults(input) {
      var srcEvent = input.srcEvent;
      var direction = input.offsetDirection; // if the touch action did prevented once this session

      if (this.manager.session.prevented) {
        srcEvent.preventDefault();
        return;
      }

      var actions = this.actions;
      var hasNone = inStr(actions, TOUCH_ACTION_NONE) && !TOUCH_ACTION_MAP[TOUCH_ACTION_NONE];
      var hasPanY = inStr(actions, TOUCH_ACTION_PAN_Y) && !TOUCH_ACTION_MAP[TOUCH_ACTION_PAN_Y];
      var hasPanX = inStr(actions, TOUCH_ACTION_PAN_X) && !TOUCH_ACTION_MAP[TOUCH_ACTION_PAN_X];

      if (hasNone) {
        //do not prevent defaults if this is a tap gesture
        var isTapPointer = input.pointers.length === 1;
        var isTapMovement = input.distance < 2;
        var isTapTouchTime = input.deltaTime < 250;

        if (isTapPointer && isTapMovement && isTapTouchTime) {
          return;
        }
      }

      if (hasPanX && hasPanY) {
        // `pan-x pan-y` means browser handles all scrolling/panning, do not prevent
        return;
      }

      if (hasNone || hasPanY && direction & DIRECTION_HORIZONTAL || hasPanX && direction & DIRECTION_VERTICAL) {
        return this.preventSrc(srcEvent);
      }
    },

    /**
     * call preventDefault to prevent the browser's default behavior (scrolling in most cases)
     * @param {Object} srcEvent
     */
    preventSrc: function preventSrc(srcEvent) {
      this.manager.session.prevented = true;
      srcEvent.preventDefault();
    }
  };
  /**
   * when the touchActions are collected they are not a valid value, so we need to clean things up. *
   * @param {String} actions
   * @returns {*}
   */

  function cleanTouchActions(actions) {
    // none
    if (inStr(actions, TOUCH_ACTION_NONE)) {
      return TOUCH_ACTION_NONE;
    }

    var hasPanX = inStr(actions, TOUCH_ACTION_PAN_X);
    var hasPanY = inStr(actions, TOUCH_ACTION_PAN_Y); // if both pan-x and pan-y are set (different recognizers
    // for different directions, e.g. horizontal pan but vertical swipe?)
    // we need none (as otherwise with pan-x pan-y combined none of these
    // recognizers will work, since the browser would handle all panning

    if (hasPanX && hasPanY) {
      return TOUCH_ACTION_NONE;
    } // pan-x OR pan-y


    if (hasPanX || hasPanY) {
      return hasPanX ? TOUCH_ACTION_PAN_X : TOUCH_ACTION_PAN_Y;
    } // manipulation


    if (inStr(actions, TOUCH_ACTION_MANIPULATION)) {
      return TOUCH_ACTION_MANIPULATION;
    }

    return TOUCH_ACTION_AUTO;
  }

  function getTouchActionProps() {
    if (!NATIVE_TOUCH_ACTION) {
      return false;
    }

    var touchMap = {};
    var cssSupports = window.CSS && window.CSS.supports;
    ['auto', 'manipulation', 'pan-y', 'pan-x', 'pan-x pan-y', 'none'].forEach(function (val) {
      // If css.supports is not supported but there is native touch-action assume it supports
      // all values. This is the case for IE 10 and 11.
      touchMap[val] = cssSupports ? window.CSS.supports('touch-action', val) : true;
    });
    return touchMap;
  }
  /**
   * Recognizer flow explained; *
   * All recognizers have the initial state of POSSIBLE when a input session starts.
   * The definition of a input session is from the first input until the last input, with all it's movement in it. *
   * Example session for mouse-input: mousedown -> mousemove -> mouseup
   *
   * On each recognizing cycle (see Manager.recognize) the .recognize() method is executed
   * which determines with state it should be.
   *
   * If the recognizer has the state FAILED, CANCELLED or RECOGNIZED (equals ENDED), it is reset to
   * POSSIBLE to give it another change on the next cycle.
   *
   *               Possible
   *                  |
   *            +-----+---------------+
   *            |                     |
   *      +-----+-----+               |
   *      |           |               |
   *   Failed      Cancelled          |
   *                          +-------+------+
   *                          |              |
   *                      Recognized       Began
   *                                         |
   *                                      Changed
   *                                         |
   *                                  Ended/Recognized
   */


  var STATE_POSSIBLE = 1;
  var STATE_BEGAN = 2;
  var STATE_CHANGED = 4;
  var STATE_ENDED = 8;
  var STATE_RECOGNIZED = STATE_ENDED;
  var STATE_CANCELLED = 16;
  var STATE_FAILED = 32;
  /**
   * Recognizer
   * Every recognizer needs to extend from this class.
   * @constructor
   * @param {Object} options
   */

  function Recognizer(options) {
    this.options = assign({}, this.defaults, options || {});
    this.id = uniqueId();
    this.manager = null; // default is enable true

    this.options.enable = ifUndefined(this.options.enable, true);
    this.state = STATE_POSSIBLE;
    this.simultaneous = {};
    this.requireFail = [];
  }

  Recognizer.prototype = {
    /**
     * @virtual
     * @type {Object}
     */
    defaults: {},

    /**
     * set options
     * @param {Object} options
     * @return {Recognizer}
     */
    set: function set(options) {
      assign(this.options, options); // also update the touchAction, in case something changed about the directions/enabled state

      this.manager && this.manager.touchAction.update();
      return this;
    },

    /**
     * recognize simultaneous with an other recognizer.
     * @param {Recognizer} otherRecognizer
     * @returns {Recognizer} this
     */
    recognizeWith: function recognizeWith(otherRecognizer) {
      if (invokeArrayArg(otherRecognizer, 'recognizeWith', this)) {
        return this;
      }

      var simultaneous = this.simultaneous;
      otherRecognizer = getRecognizerByNameIfManager(otherRecognizer, this);

      if (!simultaneous[otherRecognizer.id]) {
        simultaneous[otherRecognizer.id] = otherRecognizer;
        otherRecognizer.recognizeWith(this);
      }

      return this;
    },

    /**
     * drop the simultaneous link. it doesnt remove the link on the other recognizer.
     * @param {Recognizer} otherRecognizer
     * @returns {Recognizer} this
     */
    dropRecognizeWith: function dropRecognizeWith(otherRecognizer) {
      if (invokeArrayArg(otherRecognizer, 'dropRecognizeWith', this)) {
        return this;
      }

      otherRecognizer = getRecognizerByNameIfManager(otherRecognizer, this);
      delete this.simultaneous[otherRecognizer.id];
      return this;
    },

    /**
     * recognizer can only run when an other is failing
     * @param {Recognizer} otherRecognizer
     * @returns {Recognizer} this
     */
    requireFailure: function requireFailure(otherRecognizer) {
      if (invokeArrayArg(otherRecognizer, 'requireFailure', this)) {
        return this;
      }

      var requireFail = this.requireFail;
      otherRecognizer = getRecognizerByNameIfManager(otherRecognizer, this);

      if (inArray(requireFail, otherRecognizer) === -1) {
        requireFail.push(otherRecognizer);
        otherRecognizer.requireFailure(this);
      }

      return this;
    },

    /**
     * drop the requireFailure link. it does not remove the link on the other recognizer.
     * @param {Recognizer} otherRecognizer
     * @returns {Recognizer} this
     */
    dropRequireFailure: function dropRequireFailure(otherRecognizer) {
      if (invokeArrayArg(otherRecognizer, 'dropRequireFailure', this)) {
        return this;
      }

      otherRecognizer = getRecognizerByNameIfManager(otherRecognizer, this);
      var index = inArray(this.requireFail, otherRecognizer);

      if (index > -1) {
        this.requireFail.splice(index, 1);
      }

      return this;
    },

    /**
     * has require failures boolean
     * @returns {boolean}
     */
    hasRequireFailures: function hasRequireFailures() {
      return this.requireFail.length > 0;
    },

    /**
     * if the recognizer can recognize simultaneous with an other recognizer
     * @param {Recognizer} otherRecognizer
     * @returns {Boolean}
     */
    canRecognizeWith: function canRecognizeWith(otherRecognizer) {
      return !!this.simultaneous[otherRecognizer.id];
    },

    /**
     * You should use `tryEmit` instead of `emit` directly to check
     * that all the needed recognizers has failed before emitting.
     * @param {Object} input
     */
    emit: function emit(input) {
      var self = this;
      var state = this.state;

      function emit(event) {
        self.manager.emit(event, input);
      } // 'panstart' and 'panmove'


      if (state < STATE_ENDED) {
        emit(self.options.event + stateStr(state));
      }

      emit(self.options.event); // simple 'eventName' events

      if (input.additionalEvent) {
        // additional event(panleft, panright, pinchin, pinchout...)
        emit(input.additionalEvent);
      } // panend and pancancel


      if (state >= STATE_ENDED) {
        emit(self.options.event + stateStr(state));
      }
    },

    /**
     * Check that all the require failure recognizers has failed,
     * if true, it emits a gesture event,
     * otherwise, setup the state to FAILED.
     * @param {Object} input
     */
    tryEmit: function tryEmit(input) {
      if (this.canEmit()) {
        return this.emit(input);
      } // it's failing anyway


      this.state = STATE_FAILED;
    },

    /**
     * can we emit?
     * @returns {boolean}
     */
    canEmit: function canEmit() {
      var i = 0;

      while (i < this.requireFail.length) {
        if (!(this.requireFail[i].state & (STATE_FAILED | STATE_POSSIBLE))) {
          return false;
        }

        i++;
      }

      return true;
    },

    /**
     * update the recognizer
     * @param {Object} inputData
     */
    recognize: function recognize(inputData) {
      // make a new copy of the inputData
      // so we can change the inputData without messing up the other recognizers
      var inputDataClone = assign({}, inputData); // is is enabled and allow recognizing?

      if (!boolOrFn(this.options.enable, [this, inputDataClone])) {
        this.reset();
        this.state = STATE_FAILED;
        return;
      } // reset when we've reached the end


      if (this.state & (STATE_RECOGNIZED | STATE_CANCELLED | STATE_FAILED)) {
        this.state = STATE_POSSIBLE;
      }

      this.state = this.process(inputDataClone); // the recognizer has recognized a gesture
      // so trigger an event

      if (this.state & (STATE_BEGAN | STATE_CHANGED | STATE_ENDED | STATE_CANCELLED)) {
        this.tryEmit(inputDataClone);
      }
    },

    /**
     * return the state of the recognizer
     * the actual recognizing happens in this method
     * @virtual
     * @param {Object} inputData
     * @returns {Const} STATE
     */
    process: function process(inputData) {},
    // jshint ignore:line

    /**
     * return the preferred touch-action
     * @virtual
     * @returns {Array}
     */
    getTouchAction: function getTouchAction() {},

    /**
     * called when the gesture isn't allowed to recognize
     * like when another is being recognized or it is disabled
     * @virtual
     */
    reset: function reset() {}
  };
  /**
   * get a usable string, used as event postfix
   * @param {Const} state
   * @returns {String} state
   */

  function stateStr(state) {
    if (state & STATE_CANCELLED) {
      return 'cancel';
    } else if (state & STATE_ENDED) {
      return 'end';
    } else if (state & STATE_CHANGED) {
      return 'move';
    } else if (state & STATE_BEGAN) {
      return 'start';
    }

    return '';
  }
  /**
   * direction cons to string
   * @param {Const} direction
   * @returns {String}
   */


  function directionStr(direction) {
    if (direction == DIRECTION_DOWN) {
      return 'down';
    } else if (direction == DIRECTION_UP) {
      return 'up';
    } else if (direction == DIRECTION_LEFT) {
      return 'left';
    } else if (direction == DIRECTION_RIGHT) {
      return 'right';
    }

    return '';
  }
  /**
   * get a recognizer by name if it is bound to a manager
   * @param {Recognizer|String} otherRecognizer
   * @param {Recognizer} recognizer
   * @returns {Recognizer}
   */


  function getRecognizerByNameIfManager(otherRecognizer, recognizer) {
    var manager = recognizer.manager;

    if (manager) {
      return manager.get(otherRecognizer);
    }

    return otherRecognizer;
  }
  /**
   * This recognizer is just used as a base for the simple attribute recognizers.
   * @constructor
   * @extends Recognizer
   */


  function AttrRecognizer() {
    Recognizer.apply(this, arguments);
  }

  inherit(AttrRecognizer, Recognizer, {
    /**
     * @namespace
     * @memberof AttrRecognizer
     */
    defaults: {
      /**
       * @type {Number}
       * @default 1
       */
      pointers: 1
    },

    /**
     * Used to check if it the recognizer receives valid input, like input.distance > 10.
     * @memberof AttrRecognizer
     * @param {Object} input
     * @returns {Boolean} recognized
     */
    attrTest: function attrTest(input) {
      var optionPointers = this.options.pointers;
      return optionPointers === 0 || input.pointers.length === optionPointers;
    },

    /**
     * Process the input and return the state for the recognizer
     * @memberof AttrRecognizer
     * @param {Object} input
     * @returns {*} State
     */
    process: function process(input) {
      var state = this.state;
      var eventType = input.eventType;
      var isRecognized = state & (STATE_BEGAN | STATE_CHANGED);
      var isValid = this.attrTest(input); // on cancel input and we've recognized before, return STATE_CANCELLED

      if (isRecognized && (eventType & INPUT_CANCEL || !isValid)) {
        return state | STATE_CANCELLED;
      } else if (isRecognized || isValid) {
        if (eventType & INPUT_END) {
          return state | STATE_ENDED;
        } else if (!(state & STATE_BEGAN)) {
          return STATE_BEGAN;
        }

        return state | STATE_CHANGED;
      }

      return STATE_FAILED;
    }
  });
  /**
   * Pan
   * Recognized when the pointer is down and moved in the allowed direction.
   * @constructor
   * @extends AttrRecognizer
   */

  function PanRecognizer() {
    AttrRecognizer.apply(this, arguments);
    this.pX = null;
    this.pY = null;
  }

  inherit(PanRecognizer, AttrRecognizer, {
    /**
     * @namespace
     * @memberof PanRecognizer
     */
    defaults: {
      event: 'pan',
      threshold: 10,
      pointers: 1,
      direction: DIRECTION_ALL
    },
    getTouchAction: function getTouchAction() {
      var direction = this.options.direction;
      var actions = [];

      if (direction & DIRECTION_HORIZONTAL) {
        actions.push(TOUCH_ACTION_PAN_Y);
      }

      if (direction & DIRECTION_VERTICAL) {
        actions.push(TOUCH_ACTION_PAN_X);
      }

      return actions;
    },
    directionTest: function directionTest(input) {
      var options = this.options;
      var hasMoved = true;
      var distance = input.distance;
      var direction = input.direction;
      var x = input.deltaX;
      var y = input.deltaY; // lock to axis?

      if (!(direction & options.direction)) {
        if (options.direction & DIRECTION_HORIZONTAL) {
          direction = x === 0 ? DIRECTION_NONE : x < 0 ? DIRECTION_LEFT : DIRECTION_RIGHT;
          hasMoved = x != this.pX;
          distance = Math.abs(input.deltaX);
        } else {
          direction = y === 0 ? DIRECTION_NONE : y < 0 ? DIRECTION_UP : DIRECTION_DOWN;
          hasMoved = y != this.pY;
          distance = Math.abs(input.deltaY);
        }
      }

      input.direction = direction;
      return hasMoved && distance > options.threshold && direction & options.direction;
    },
    attrTest: function attrTest(input) {
      return AttrRecognizer.prototype.attrTest.call(this, input) && (this.state & STATE_BEGAN || !(this.state & STATE_BEGAN) && this.directionTest(input));
    },
    emit: function emit(input) {
      this.pX = input.deltaX;
      this.pY = input.deltaY;
      var direction = directionStr(input.direction);

      if (direction) {
        input.additionalEvent = this.options.event + direction;
      }

      this._super.emit.call(this, input);
    }
  });
  /**
   * Pinch
   * Recognized when two or more pointers are moving toward (zoom-in) or away from each other (zoom-out).
   * @constructor
   * @extends AttrRecognizer
   */

  function PinchRecognizer() {
    AttrRecognizer.apply(this, arguments);
  }

  inherit(PinchRecognizer, AttrRecognizer, {
    /**
     * @namespace
     * @memberof PinchRecognizer
     */
    defaults: {
      event: 'pinch',
      threshold: 0,
      pointers: 2
    },
    getTouchAction: function getTouchAction() {
      return [TOUCH_ACTION_NONE];
    },
    attrTest: function attrTest(input) {
      return this._super.attrTest.call(this, input) && (Math.abs(input.scale - 1) > this.options.threshold || this.state & STATE_BEGAN);
    },
    emit: function emit(input) {
      if (input.scale !== 1) {
        var inOut = input.scale < 1 ? 'in' : 'out';
        input.additionalEvent = this.options.event + inOut;
      }

      this._super.emit.call(this, input);
    }
  });
  /**
   * Press
   * Recognized when the pointer is down for x ms without any movement.
   * @constructor
   * @extends Recognizer
   */

  function PressRecognizer() {
    Recognizer.apply(this, arguments);
    this._timer = null;
    this._input = null;
  }

  inherit(PressRecognizer, Recognizer, {
    /**
     * @namespace
     * @memberof PressRecognizer
     */
    defaults: {
      event: 'press',
      pointers: 1,
      time: 251,
      // minimal time of the pointer to be pressed
      threshold: 9 // a minimal movement is ok, but keep it low

    },
    getTouchAction: function getTouchAction() {
      return [TOUCH_ACTION_AUTO];
    },
    process: function process(input) {
      var options = this.options;
      var validPointers = input.pointers.length === options.pointers;
      var validMovement = input.distance < options.threshold;
      var validTime = input.deltaTime > options.time;
      this._input = input; // we only allow little movement
      // and we've reached an end event, so a tap is possible

      if (!validMovement || !validPointers || input.eventType & (INPUT_END | INPUT_CANCEL) && !validTime) {
        this.reset();
      } else if (input.eventType & INPUT_START) {
        this.reset();
        this._timer = setTimeoutContext(function () {
          this.state = STATE_RECOGNIZED;
          this.tryEmit();
        }, options.time, this);
      } else if (input.eventType & INPUT_END) {
        return STATE_RECOGNIZED;
      }

      return STATE_FAILED;
    },
    reset: function reset() {
      clearTimeout(this._timer);
    },
    emit: function emit(input) {
      if (this.state !== STATE_RECOGNIZED) {
        return;
      }

      if (input && input.eventType & INPUT_END) {
        this.manager.emit(this.options.event + 'up', input);
      } else {
        this._input.timeStamp = now();
        this.manager.emit(this.options.event, this._input);
      }
    }
  });
  /**
   * Rotate
   * Recognized when two or more pointer are moving in a circular motion.
   * @constructor
   * @extends AttrRecognizer
   */

  function RotateRecognizer() {
    AttrRecognizer.apply(this, arguments);
  }

  inherit(RotateRecognizer, AttrRecognizer, {
    /**
     * @namespace
     * @memberof RotateRecognizer
     */
    defaults: {
      event: 'rotate',
      threshold: 0,
      pointers: 2
    },
    getTouchAction: function getTouchAction() {
      return [TOUCH_ACTION_NONE];
    },
    attrTest: function attrTest(input) {
      return this._super.attrTest.call(this, input) && (Math.abs(input.rotation) > this.options.threshold || this.state & STATE_BEGAN);
    }
  });
  /**
   * Swipe
   * Recognized when the pointer is moving fast (velocity), with enough distance in the allowed direction.
   * @constructor
   * @extends AttrRecognizer
   */

  function SwipeRecognizer() {
    AttrRecognizer.apply(this, arguments);
  }

  inherit(SwipeRecognizer, AttrRecognizer, {
    /**
     * @namespace
     * @memberof SwipeRecognizer
     */
    defaults: {
      event: 'swipe',
      threshold: 10,
      velocity: 0.3,
      direction: DIRECTION_HORIZONTAL | DIRECTION_VERTICAL,
      pointers: 1
    },
    getTouchAction: function getTouchAction() {
      return PanRecognizer.prototype.getTouchAction.call(this);
    },
    attrTest: function attrTest(input) {
      var direction = this.options.direction;
      var velocity;

      if (direction & (DIRECTION_HORIZONTAL | DIRECTION_VERTICAL)) {
        velocity = input.overallVelocity;
      } else if (direction & DIRECTION_HORIZONTAL) {
        velocity = input.overallVelocityX;
      } else if (direction & DIRECTION_VERTICAL) {
        velocity = input.overallVelocityY;
      }

      return this._super.attrTest.call(this, input) && direction & input.offsetDirection && input.distance > this.options.threshold && input.maxPointers == this.options.pointers && abs(velocity) > this.options.velocity && input.eventType & INPUT_END;
    },
    emit: function emit(input) {
      var direction = directionStr(input.offsetDirection);

      if (direction) {
        this.manager.emit(this.options.event + direction, input);
      }

      this.manager.emit(this.options.event, input);
    }
  });
  /**
   * A tap is ecognized when the pointer is doing a small tap/click. Multiple taps are recognized if they occur
   * between the given interval and position. The delay option can be used to recognize multi-taps without firing
   * a single tap.
   *
   * The eventData from the emitted event contains the property `tapCount`, which contains the amount of
   * multi-taps being recognized.
   * @constructor
   * @extends Recognizer
   */

  function TapRecognizer() {
    Recognizer.apply(this, arguments); // previous time and center,
    // used for tap counting

    this.pTime = false;
    this.pCenter = false;
    this._timer = null;
    this._input = null;
    this.count = 0;
  }

  inherit(TapRecognizer, Recognizer, {
    /**
     * @namespace
     * @memberof PinchRecognizer
     */
    defaults: {
      event: 'tap',
      pointers: 1,
      taps: 1,
      interval: 300,
      // max time between the multi-tap taps
      time: 250,
      // max time of the pointer to be down (like finger on the screen)
      threshold: 9,
      // a minimal movement is ok, but keep it low
      posThreshold: 10 // a multi-tap can be a bit off the initial position

    },
    getTouchAction: function getTouchAction() {
      return [TOUCH_ACTION_MANIPULATION];
    },
    process: function process(input) {
      var options = this.options;
      var validPointers = input.pointers.length === options.pointers;
      var validMovement = input.distance < options.threshold;
      var validTouchTime = input.deltaTime < options.time;
      this.reset();

      if (input.eventType & INPUT_START && this.count === 0) {
        return this.failTimeout();
      } // we only allow little movement
      // and we've reached an end event, so a tap is possible


      if (validMovement && validTouchTime && validPointers) {
        if (input.eventType != INPUT_END) {
          return this.failTimeout();
        }

        var validInterval = this.pTime ? input.timeStamp - this.pTime < options.interval : true;
        var validMultiTap = !this.pCenter || getDistance(this.pCenter, input.center) < options.posThreshold;
        this.pTime = input.timeStamp;
        this.pCenter = input.center;

        if (!validMultiTap || !validInterval) {
          this.count = 1;
        } else {
          this.count += 1;
        }

        this._input = input; // if tap count matches we have recognized it,
        // else it has began recognizing...

        var tapCount = this.count % options.taps;

        if (tapCount === 0) {
          // no failing requirements, immediately trigger the tap event
          // or wait as long as the multitap interval to trigger
          if (!this.hasRequireFailures()) {
            return STATE_RECOGNIZED;
          } else {
            this._timer = setTimeoutContext(function () {
              this.state = STATE_RECOGNIZED;
              this.tryEmit();
            }, options.interval, this);
            return STATE_BEGAN;
          }
        }
      }

      return STATE_FAILED;
    },
    failTimeout: function failTimeout() {
      this._timer = setTimeoutContext(function () {
        this.state = STATE_FAILED;
      }, this.options.interval, this);
      return STATE_FAILED;
    },
    reset: function reset() {
      clearTimeout(this._timer);
    },
    emit: function emit() {
      if (this.state == STATE_RECOGNIZED) {
        this._input.tapCount = this.count;
        this.manager.emit(this.options.event, this._input);
      }
    }
  });
  /**
   * Simple way to create a manager with a default set of recognizers.
   * @param {HTMLElement} element
   * @param {Object} [options]
   * @constructor
   */

  function Hammer(element, options) {
    options = options || {};
    options.recognizers = ifUndefined(options.recognizers, Hammer.defaults.preset);
    return new Manager(element, options);
  }
  /**
   * @const {string}
   */


  Hammer.VERSION = '2.0.7';
  /**
   * default settings
   * @namespace
   */

  Hammer.defaults = {
    /**
     * set if DOM events are being triggered.
     * But this is slower and unused by simple implementations, so disabled by default.
     * @type {Boolean}
     * @default false
     */
    domEvents: false,

    /**
     * The value for the touchAction property/fallback.
     * When set to `compute` it will magically set the correct value based on the added recognizers.
     * @type {String}
     * @default compute
     */
    touchAction: TOUCH_ACTION_COMPUTE,

    /**
     * @type {Boolean}
     * @default true
     */
    enable: true,

    /**
     * EXPERIMENTAL FEATURE -- can be removed/changed
     * Change the parent input target element.
     * If Null, then it is being set the to main element.
     * @type {Null|EventTarget}
     * @default null
     */
    inputTarget: null,

    /**
     * force an input class
     * @type {Null|Function}
     * @default null
     */
    inputClass: null,

    /**
     * Default recognizer setup when calling `Hammer()`
     * When creating a new Manager these will be skipped.
     * @type {Array}
     */
    preset: [// RecognizerClass, options, [recognizeWith, ...], [requireFailure, ...]
    [RotateRecognizer, {
      enable: false
    }], [PinchRecognizer, {
      enable: false
    }, ['rotate']], [SwipeRecognizer, {
      direction: DIRECTION_HORIZONTAL
    }], [PanRecognizer, {
      direction: DIRECTION_HORIZONTAL
    }, ['swipe']], [TapRecognizer], [TapRecognizer, {
      event: 'doubletap',
      taps: 2
    }, ['tap']], [PressRecognizer]],

    /**
     * Some CSS properties can be used to improve the working of Hammer.
     * Add them to this method and they will be set when creating a new Manager.
     * @namespace
     */
    cssProps: {
      /**
       * Disables text selection to improve the dragging gesture. Mainly for desktop browsers.
       * @type {String}
       * @default 'none'
       */
      userSelect: 'none',

      /**
       * Disable the Windows Phone grippers when pressing an element.
       * @type {String}
       * @default 'none'
       */
      touchSelect: 'none',

      /**
       * Disables the default callout shown when you touch and hold a touch target.
       * On iOS, when you touch and hold a touch target such as a link, Safari displays
       * a callout containing information about the link. This property allows you to disable that callout.
       * @type {String}
       * @default 'none'
       */
      touchCallout: 'none',

      /**
       * Specifies whether zooming is enabled. Used by IE10>
       * @type {String}
       * @default 'none'
       */
      contentZooming: 'none',

      /**
       * Specifies that an entire element should be draggable instead of its contents. Mainly for desktop browsers.
       * @type {String}
       * @default 'none'
       */
      userDrag: 'none',

      /**
       * Overrides the highlight color shown when the user taps a link or a JavaScript
       * clickable element in iOS. This property obeys the alpha value, if specified.
       * @type {String}
       * @default 'rgba(0,0,0,0)'
       */
      tapHighlightColor: 'rgba(0,0,0,0)'
    }
  };
  var STOP = 1;
  var FORCED_STOP = 2;
  /**
   * Manager
   * @param {HTMLElement} element
   * @param {Object} [options]
   * @constructor
   */

  function Manager(element, options) {
    this.options = assign({}, Hammer.defaults, options || {});
    this.options.inputTarget = this.options.inputTarget || element;
    this.handlers = {};
    this.session = {};
    this.recognizers = [];
    this.oldCssProps = {};
    this.element = element;
    this.input = createInputInstance(this);
    this.touchAction = new TouchAction(this, this.options.touchAction);
    toggleCssProps(this, true);
    each(this.options.recognizers, function (item) {
      var recognizer = this.add(new item[0](item[1]));
      item[2] && recognizer.recognizeWith(item[2]);
      item[3] && recognizer.requireFailure(item[3]);
    }, this);
  }

  Manager.prototype = {
    /**
     * set options
     * @param {Object} options
     * @returns {Manager}
     */
    set: function set(options) {
      assign(this.options, options); // Options that need a little more setup

      if (options.touchAction) {
        this.touchAction.update();
      }

      if (options.inputTarget) {
        // Clean up existing event listeners and reinitialize
        this.input.destroy();
        this.input.target = options.inputTarget;
        this.input.init();
      }

      return this;
    },

    /**
     * stop recognizing for this session.
     * This session will be discarded, when a new [input]start event is fired.
     * When forced, the recognizer cycle is stopped immediately.
     * @param {Boolean} [force]
     */
    stop: function stop(force) {
      this.session.stopped = force ? FORCED_STOP : STOP;
    },

    /**
     * run the recognizers!
     * called by the inputHandler function on every movement of the pointers (touches)
     * it walks through all the recognizers and tries to detect the gesture that is being made
     * @param {Object} inputData
     */
    recognize: function recognize(inputData) {
      var session = this.session;

      if (session.stopped) {
        return;
      } // run the touch-action polyfill


      this.touchAction.preventDefaults(inputData);
      var recognizer;
      var recognizers = this.recognizers; // this holds the recognizer that is being recognized.
      // so the recognizer's state needs to be BEGAN, CHANGED, ENDED or RECOGNIZED
      // if no recognizer is detecting a thing, it is set to `null`

      var curRecognizer = session.curRecognizer; // reset when the last recognizer is recognized
      // or when we're in a new session

      if (!curRecognizer || curRecognizer && curRecognizer.state & STATE_RECOGNIZED) {
        curRecognizer = session.curRecognizer = null;
      }

      var i = 0;

      while (i < recognizers.length) {
        recognizer = recognizers[i]; // find out if we are allowed try to recognize the input for this one.
        // 1.   allow if the session is NOT forced stopped (see the .stop() method)
        // 2.   allow if we still haven't recognized a gesture in this session, or the this recognizer is the one
        //      that is being recognized.
        // 3.   allow if the recognizer is allowed to run simultaneous with the current recognized recognizer.
        //      this can be setup with the `recognizeWith()` method on the recognizer.

        if (session.stopped !== FORCED_STOP && ( // 1
        !curRecognizer || recognizer == curRecognizer || // 2
        recognizer.canRecognizeWith(curRecognizer))) {
          // 3
          recognizer.recognize(inputData);
        } else {
          recognizer.reset();
        } // if the recognizer has been recognizing the input as a valid gesture, we want to store this one as the
        // current active recognizer. but only if we don't already have an active recognizer


        if (!curRecognizer && recognizer.state & (STATE_BEGAN | STATE_CHANGED | STATE_ENDED)) {
          curRecognizer = session.curRecognizer = recognizer;
        }

        i++;
      }
    },

    /**
     * get a recognizer by its event name.
     * @param {Recognizer|String} recognizer
     * @returns {Recognizer|Null}
     */
    get: function get(recognizer) {
      if (recognizer instanceof Recognizer) {
        return recognizer;
      }

      var recognizers = this.recognizers;

      for (var i = 0; i < recognizers.length; i++) {
        if (recognizers[i].options.event == recognizer) {
          return recognizers[i];
        }
      }

      return null;
    },

    /**
     * add a recognizer to the manager
     * existing recognizers with the same event name will be removed
     * @param {Recognizer} recognizer
     * @returns {Recognizer|Manager}
     */
    add: function add(recognizer) {
      if (invokeArrayArg(recognizer, 'add', this)) {
        return this;
      } // remove existing


      var existing = this.get(recognizer.options.event);

      if (existing) {
        this.remove(existing);
      }

      this.recognizers.push(recognizer);
      recognizer.manager = this;
      this.touchAction.update();
      return recognizer;
    },

    /**
     * remove a recognizer by name or instance
     * @param {Recognizer|String} recognizer
     * @returns {Manager}
     */
    remove: function remove(recognizer) {
      if (invokeArrayArg(recognizer, 'remove', this)) {
        return this;
      }

      recognizer = this.get(recognizer); // let's make sure this recognizer exists

      if (recognizer) {
        var recognizers = this.recognizers;
        var index = inArray(recognizers, recognizer);

        if (index !== -1) {
          recognizers.splice(index, 1);
          this.touchAction.update();
        }
      }

      return this;
    },

    /**
     * bind event
     * @param {String} events
     * @param {Function} handler
     * @returns {EventEmitter} this
     */
    on: function on(events, handler) {
      if (events === undefined$1) {
        return;
      }

      if (handler === undefined$1) {
        return;
      }

      var handlers = this.handlers;
      each(splitStr(events), function (event) {
        handlers[event] = handlers[event] || [];
        handlers[event].push(handler);
      });
      return this;
    },

    /**
     * unbind event, leave emit blank to remove all handlers
     * @param {String} events
     * @param {Function} [handler]
     * @returns {EventEmitter} this
     */
    off: function off(events, handler) {
      if (events === undefined$1) {
        return;
      }

      var handlers = this.handlers;
      each(splitStr(events), function (event) {
        if (!handler) {
          delete handlers[event];
        } else {
          handlers[event] && handlers[event].splice(inArray(handlers[event], handler), 1);
        }
      });
      return this;
    },

    /**
     * emit event to the listeners
     * @param {String} event
     * @param {Object} data
     */
    emit: function emit(event, data) {
      // we also want to trigger dom events
      if (this.options.domEvents) {
        triggerDomEvent(event, data);
      } // no handlers, so skip it all


      var handlers = this.handlers[event] && this.handlers[event].slice();

      if (!handlers || !handlers.length) {
        return;
      }

      data.type = event;

      data.preventDefault = function () {
        data.srcEvent.preventDefault();
      };

      var i = 0;

      while (i < handlers.length) {
        handlers[i](data);
        i++;
      }
    },

    /**
     * destroy the manager and unbinds all events
     * it doesn't unbind dom events, that is the user own responsibility
     */
    destroy: function destroy() {
      this.element && toggleCssProps(this, false);
      this.handlers = {};
      this.session = {};
      this.input.destroy();
      this.element = null;
    }
  };
  /**
   * add/remove the css properties as defined in manager.options.cssProps
   * @param {Manager} manager
   * @param {Boolean} add
   */

  function toggleCssProps(manager, add) {
    var element = manager.element;

    if (!element.style) {
      return;
    }

    var prop;
    each(manager.options.cssProps, function (value, name) {
      prop = prefixed(element.style, name);

      if (add) {
        manager.oldCssProps[prop] = element.style[prop];
        element.style[prop] = value;
      } else {
        element.style[prop] = manager.oldCssProps[prop] || '';
      }
    });

    if (!add) {
      manager.oldCssProps = {};
    }
  }
  /**
   * trigger dom event
   * @param {String} event
   * @param {Object} data
   */


  function triggerDomEvent(event, data) {
    var gestureEvent = document.createEvent('Event');
    gestureEvent.initEvent(event, true, true);
    gestureEvent.gesture = data;
    data.target.dispatchEvent(gestureEvent);
  }

  assign(Hammer, {
    INPUT_START: INPUT_START,
    INPUT_MOVE: INPUT_MOVE,
    INPUT_END: INPUT_END,
    INPUT_CANCEL: INPUT_CANCEL,
    STATE_POSSIBLE: STATE_POSSIBLE,
    STATE_BEGAN: STATE_BEGAN,
    STATE_CHANGED: STATE_CHANGED,
    STATE_ENDED: STATE_ENDED,
    STATE_RECOGNIZED: STATE_RECOGNIZED,
    STATE_CANCELLED: STATE_CANCELLED,
    STATE_FAILED: STATE_FAILED,
    DIRECTION_NONE: DIRECTION_NONE,
    DIRECTION_LEFT: DIRECTION_LEFT,
    DIRECTION_RIGHT: DIRECTION_RIGHT,
    DIRECTION_UP: DIRECTION_UP,
    DIRECTION_DOWN: DIRECTION_DOWN,
    DIRECTION_HORIZONTAL: DIRECTION_HORIZONTAL,
    DIRECTION_VERTICAL: DIRECTION_VERTICAL,
    DIRECTION_ALL: DIRECTION_ALL,
    Manager: Manager,
    Input: Input,
    TouchAction: TouchAction,
    TouchInput: TouchInput,
    MouseInput: MouseInput,
    PointerEventInput: PointerEventInput,
    TouchMouseInput: TouchMouseInput,
    SingleTouchInput: SingleTouchInput,
    Recognizer: Recognizer,
    AttrRecognizer: AttrRecognizer,
    Tap: TapRecognizer,
    Pan: PanRecognizer,
    Swipe: SwipeRecognizer,
    Pinch: PinchRecognizer,
    Rotate: RotateRecognizer,
    Press: PressRecognizer,
    on: addEventListeners,
    off: removeEventListeners,
    each: each,
    merge: merge,
    extend: extend,
    assign: assign,
    inherit: inherit,
    bindFn: bindFn,
    prefixed: prefixed
  }); // this prevents errors when Hammer is loaded in the presence of an AMD
  //  style loader but by script tag, not by the loader.

  var freeGlobal = typeof window !== 'undefined' ? window : typeof self !== 'undefined' ? self : {}; // jshint ignore:line

  freeGlobal.Hammer = Hammer;

  if (typeof define === 'function' && define.amd) {
    define(function () {
      return Hammer;
    });
  } else if (typeof module != 'undefined' && module.exports) {
    module.exports = Hammer;
  } else {
    window[exportName] = Hammer;
  }
})(window, document, 'Hammer');

var Hammer = /*#__PURE__*/Object.freeze({
  __proto__: null
});

/* eslint-disable no-unused-vars */
var Cesium$1 = window.Cesium;
var knockout = Cesium$1.knockout;
var KnockoutHammerBinding = {
  register: function register(Knockout) {
    Knockout.bindingHandlers.swipeLeft = {
      init: function init(element, valueAccessor, allBindings, viewModel, bindingContext) {
        var f = Knockout.unwrap(valueAccessor());
        new Hammer(element).on('swipeleft', function (e) {
          var viewModel = bindingContext.$data;
          f.apply(viewModel, arguments);
        });
      }
    };
    Knockout.bindingHandlers.swipeRight = {
      init: function init(element, valueAccessor, allBindings, viewModel, bindingContext) {
        var f = Knockout.unwrap(valueAccessor());
        new Hammer(element).on('swiperight', function (e) {
          var viewModel = bindingContext.$data;
          f.apply(viewModel, arguments);
        });
      }
    };
  }
};

/* eslint-disable no-unused-vars */
var Cesium$2 = window.Cesium;
var knockout$1 = Cesium$2.knockout,
    SvgPathBindingHandler = Cesium$2.SvgPathBindingHandler;
var Knockout = knockout$1;

var registerKnockoutBindings = function registerKnockoutBindings() {
  SvgPathBindingHandler.register(Knockout);
  KnockoutMarkdownBinding.register(Knockout);
  KnockoutHammerBinding.register(Knockout);
  Knockout.bindingHandlers.embeddedComponent = {
    init: function init(element, valueAccessor, allBindings, viewModel, bindingContext) {
      var component = Knockout.unwrap(valueAccessor());
      component.show(element);
      return {
        controlsDescendantBindings: true
      };
    },
    update: function update(element, valueAccessor, allBindings, viewModel, bindingContext) {}
  };
};

var createFragmentFromTemplate = function createFragmentFromTemplate(htmlString) {
  var holder = document.createElement('div');
  holder.innerHTML = htmlString;
  var fragment = document.createDocumentFragment();

  while (holder.firstChild) {
    fragment.appendChild(holder.firstChild);
  }

  return fragment;
};

/* eslint-disable no-unused-vars */
var Cesium$3 = window.Cesium;
var knockout$2 = Cesium$3.knockout,
    getElement = Cesium$3.getElement;
var Knockout$1 = knockout$2;

var loadView = function loadView(htmlString, container, viewModel) {
  container = getElement(container);
  var fragment = createFragmentFromTemplate(htmlString); // Sadly, fragment.childNodes doesn't have a slice function.
  // This code could be replaced with Array.prototype.slice.call(fragment.childNodes)
  // but that seems slightly error prone.

  var nodes = [];
  var i;

  for (i = 0; i < fragment.childNodes.length; ++i) {
    nodes.push(fragment.childNodes[i]);
  }

  container.appendChild(fragment);

  for (i = 0; i < nodes.length; ++i) {
    var node = nodes[i];

    if (node.nodeType === 1 || node.nodeType === 8) {
      Knockout$1.applyBindings(viewModel, node);
    }
  }

  return nodes;
};

/* eslint-disable no-unused-vars */
var Cesium$4 = window.Cesium;
var defined = Cesium$4.defined,
    DeveloperError = Cesium$4.DeveloperError,
    EllipsoidGeodesic = Cesium$4.EllipsoidGeodesic,
    Cartesian2 = Cesium$4.Cartesian2,
    getTimestamp = Cesium$4.getTimestamp,
    EventHelper = Cesium$4.EventHelper,
    knockout$3 = Cesium$4.knockout;
var Knockout$2 = knockout$3;

var DistanceLegendViewModel = function DistanceLegendViewModel(options) {
  if (!defined(options) || !defined(options.terria)) {
    throw new DeveloperError('options.terria is required.');
  }

  this.terria = options.terria;
  this._removeSubscription = undefined;
  this._lastLegendUpdate = undefined;
  this.eventHelper = new EventHelper();
  this.distanceLabel = undefined;
  this.barWidth = undefined;
  this.enableDistanceLegend = defined(options.enableDistanceLegend) ? options.enableDistanceLegend : true;
  Knockout$2.track(this, ['distanceLabel', 'barWidth']);
  this.eventHelper.add(this.terria.afterWidgetChanged, function () {
    if (defined(this._removeSubscription)) {
      this._removeSubscription();

      this._removeSubscription = undefined;
    }
  }, this); //        this.terria.beforeWidgetChanged.addEventListener(function () {
  //            if (defined(this._removeSubscription)) {
  //                this._removeSubscription();
  //                this._removeSubscription = undefined;
  //            }
  //        }, this);

  var that = this;

  function addUpdateSubscription() {
    if (defined(that.terria)) {
      var scene = that.terria.scene;
      that._removeSubscription = scene.postRender.addEventListener(function () {
        updateDistanceLegendCesium(this, scene);
      }, that);
    }
  }

  addUpdateSubscription();
  this.eventHelper.add(this.terria.afterWidgetChanged, function () {
    addUpdateSubscription();
  }, this); // this.terria.afterWidgetChanged.addEventListener(function() {
  //    addUpdateSubscription();
  // }, this);
};

DistanceLegendViewModel.prototype.destroy = function () {
  this.eventHelper.removeAll();
};

DistanceLegendViewModel.prototype.show = function (container) {
  var testing;

  if (this.enableDistanceLegend) {
    testing = '<div class="distance-legend" id="vis3d-distance-legend" data-bind="visible: distanceLabel && barWidth">' + '<div class="distance-legend-label" data-bind="text: distanceLabel"></div>' + '<div class="distance-legend-scale-bar" data-bind="style: { width: barWidth + \'px\', left: ((125 - barWidth) / 2) + \'px\' }"></div>' + '</div>';
  } else {
    testing = '<div class="distance-legend" id="vis3d-distance-legend" style="display: none;" data-bind="visible: distanceLabel && barWidth">' + '<div class="distance-legend-label"  data-bind="text: distanceLabel"></div>' + '<div class="distance-legend-scale-bar"  data-bind="style: { width: barWidth + \'px\', left: (5 + (125 - barWidth) / 2) + \'px\' }"></div>' + '</div>';
  }

  loadView(testing, container, this);
};

DistanceLegendViewModel.create = function (options) {
  var result = new DistanceLegendViewModel(options);
  result.show(options.container);
  result.setStyle(options.style);
  return result;
}; // 设置样式


DistanceLegendViewModel.prototype.setStyle = function (style) {
  if (!style || Object.keys(style).length < 1) return;
  var ele = document.getElementById("vis3d-distance-legend");
  if (!ele) return;

  for (var i in style) {
    ele.style[i] = style[i];
  }
};

var geodesic = new EllipsoidGeodesic();
var distances = [1, 2, 3, 5, 10, 20, 30, 50, 100, 200, 300, 500, 1000, 2000, 3000, 5000, 10000, 20000, 30000, 50000, 100000, 200000, 300000, 500000, 1000000, 2000000, 3000000, 5000000, 10000000, 20000000, 30000000, 50000000];

function updateDistanceLegendCesium(viewModel, scene) {
  if (!viewModel.enableDistanceLegend) {
    viewModel.barWidth = undefined;
    viewModel.distanceLabel = undefined;
    return;
  }

  var now = getTimestamp();

  if (now < viewModel._lastLegendUpdate + 250) {
    return;
  }

  viewModel._lastLegendUpdate = now; // Find the distance between two pixels at the bottom center of the screen.

  var width = scene.canvas.clientWidth;
  var height = scene.canvas.clientHeight;
  var left = scene.camera.getPickRay(new Cartesian2(width / 2 | 0, height - 1));
  var right = scene.camera.getPickRay(new Cartesian2(1 + width / 2 | 0, height - 1));
  var globe = scene.globe;
  var leftPosition = globe.pick(left, scene);
  var rightPosition = globe.pick(right, scene);

  if (!defined(leftPosition) || !defined(rightPosition)) {
    viewModel.barWidth = undefined;
    viewModel.distanceLabel = undefined;
    return;
  }

  var leftCartographic = globe.ellipsoid.cartesianToCartographic(leftPosition);
  var rightCartographic = globe.ellipsoid.cartesianToCartographic(rightPosition);
  geodesic.setEndPoints(leftCartographic, rightCartographic);
  var pixelDistance = geodesic.surfaceDistance; // Find the first distance that makes the scale bar less than 100 pixels.

  var maxBarWidth = 100;
  var distance;

  for (var i = distances.length - 1; !defined(distance) && i >= 0; --i) {
    if (distances[i] / pixelDistance < maxBarWidth) {
      distance = distances[i];
    }
  }

  if (defined(distance)) {
    var label;

    if (distance >= 1000) {
      label = (distance / 1000).toString() + ' km';
    } else {
      label = distance.toString() + ' m';
    }

    viewModel.barWidth = distance / pixelDistance | 0;
    viewModel.distanceLabel = label;
  } else {
    viewModel.barWidth = undefined;
    viewModel.distanceLabel = undefined;
  }
}

var svgReset = 'M 7.5,0 C 3.375,0 0,3.375 0,7.5 0,11.625 3.375,15 7.5,15 c 3.46875,0 6.375,-2.4375 7.21875,-5.625 l -1.96875,0 C 12,11.53125 9.9375,13.125 7.5,13.125 4.40625,13.125 1.875,10.59375 1.875,7.5 1.875,4.40625 4.40625,1.875 7.5,1.875 c 1.59375,0 2.90625,0.65625 3.9375,1.6875 l -3,3 6.5625,0 L 15,0 12.75,2.25 C 11.4375,0.84375 9.5625,0 7.5,0 z';

var Cesium$5 = window.Cesium;
var defined$1 = Cesium$5.defined,
    DeveloperError$1 = Cesium$5.DeveloperError,
    knockout$4 = Cesium$5.knockout;
var Knockout$3 = knockout$4;
/**
 * The view-model for a control in the user interface
 *
 * @alias UserInterfaceControl
 * @constructor
 * @abstract
 *
 * @param {Terria} terria The Terria instance.
 */

var UserInterfaceControl = function UserInterfaceControl(terria) {
  if (!defined$1(terria)) {
    throw new DeveloperError$1('terria is required');
  }

  this._terria = terria;
  /**
   * Gets or sets the name of the control which is set as the controls title.
   * This property is observable.
   * @type {String}
   */

  this.name = 'Unnamed Control';
  /**
   * Gets or sets the text to be displayed in the UI control.
   * This property is observable.
   * @type {String}
   */

  this.text = undefined;
  /**
   * Gets or sets the svg icon of the control.  This property is observable.
   * @type {Object}
   */

  this.svgIcon = undefined;
  /**
   * Gets or sets the height of the svg icon.  This property is observable.
   * @type {Integer}
   */

  this.svgHeight = undefined;
  /**
   * Gets or sets the width of the svg icon.  This property is observable.
   * @type {Integer}
   */

  this.svgWidth = undefined;
  /**
   * Gets or sets the CSS class of the control. This property is observable.
   * @type {String}
   */

  this.cssClass = undefined;
  /**
   * Gets or sets the property describing whether or not the control is in the active state.
   * This property is observable.
   * @type {Boolean}
   */

  this.isActive = false;
  Knockout$3.track(this, ['name', 'svgIcon', 'svgHeight', 'svgWidth', 'cssClass', 'isActive']);
};

Object.defineProperties(UserInterfaceControl.prototype, {
  /**
   * Gets the Terria instance.
   * @memberOf UserInterfaceControl.prototype
   * @type {Terria}
   */
  terria: {
    get: function get() {
      return this._terria;
    }
  },

  /**
   * Gets a value indicating whether this button has text associated with it.
   * @type {Object}
   */
  hasText: {
    get: function get() {
      return defined$1(this.text) && typeof this.text === 'string';
    }
  }
});
/**
 * When implemented in a derived class, performs an action when the user clicks
 * on this control.
 * @abstract
 * @protected
 */

UserInterfaceControl.prototype.activate = function () {
  throw new DeveloperError$1('activate must be implemented in the derived class.');
};

/**
 * The view-model for a control in the navigation control tool bar
 *
 * @alias NavigationControl
 * @constructor
 * @abstract
 *
 * @param {Terria} terria The Terria instance.
 */

var NavigationControl = function NavigationControl(terria) {
  UserInterfaceControl.apply(this, arguments);
};

NavigationControl.prototype = Object.create(UserInterfaceControl.prototype);

var Cesium$6 = window.Cesium;
var defined$2 = Cesium$6.defined,
    Camera = Cesium$6.Camera,
    Rectangle = Cesium$6.Rectangle,
    Cartographic = Cesium$6.Cartographic,
    Math$1 = Cesium$6.Math;
/**
 * The model for a zoom in control in the navigation control tool bar
 *
 * @alias ResetViewNavigationControl
 * @constructor
 * @abstract
 *
 * @param {Terria} terria The Terria instance.
 */

var ResetViewNavigationControl = function ResetViewNavigationControl(terria) {
  NavigationControl.apply(this, arguments);
  /**
   * Gets or sets the name of the control which is set as the control's title.
   * This property is observable.
   * @type {String}
   */

  this.name = '重置视图';
  this.navigationLocked = false;
  /**
   * Gets or sets the svg icon of the control.  This property is observable.
   * @type {Object}
   */

  this.svgIcon = svgReset;
  /**
   * Gets or sets the height of the svg icon.  This property is observable.
   * @type {Integer}
   */

  this.svgHeight = 15;
  /**
   * Gets or sets the width of the svg icon.  This property is observable.
   * @type {Integer}
   */

  this.svgWidth = 15;
  /**
   * Gets or sets the CSS class of the control. This property is observable.
   * @type {String}
   */

  this.cssClass = 'navigation-control-icon-reset';
};

ResetViewNavigationControl.prototype = Object.create(NavigationControl.prototype);

ResetViewNavigationControl.prototype.setNavigationLocked = function (locked) {
  this.navigationLocked = locked;
};

ResetViewNavigationControl.prototype.resetView = function () {
  // this.terria.analytics.logEvent('navigation', 'click', 'reset');
  if (this.navigationLocked) {
    return;
  }

  var scene = this.terria.scene;
  var sscc = scene.screenSpaceCameraController;

  if (!sscc.enableInputs) {
    return;
  }

  this.isActive = true;
  var camera = scene.camera;

  if (defined$2(this.terria.trackedEntity)) {
    // when tracking do not reset to default view but to default view of tracked entity
    var trackedEntity = this.terria.trackedEntity;
    this.terria.trackedEntity = undefined;
    this.terria.trackedEntity = trackedEntity;
  } else {
    // reset to a default position or view defined in the options
    if (this.terria.options.view) {
      this.setCameraView(this.terria.options.view, this.terria);
    } else if (typeof camera.flyHome === 'function') {
      camera.flyHome(1);
    } else {
      camera.flyTo({
        'destination': Camera.DEFAULT_VIEW_RECTANGLE,
        'duration': 1
      });
    }
  }

  this.isActive = false;
};
/**
 * When implemented in a derived class, performs an action when the user clicks
 * on this control
 * @abstract
 * @protected
 */


ResetViewNavigationControl.prototype.activate = function () {
  this.resetView();
};

ResetViewNavigationControl.prototype.setCameraView = function (obj, mapViewer) {
  var viewer = mapViewer || window.viewer;
  if (!obj) return;
  var position = obj.destination || Cesium$6.Cartesian3.fromDegrees(obj.x, obj.y, obj.z); // 兼容cartesian3和xyz

  viewer.camera.flyTo({
    destination: position,
    orientation: {
      heading: Cesium$6.Math.toRadians(obj.heading || 0),
      pitch: Cesium$6.Math.toRadians(obj.pitch || 0),
      roll: Cesium$6.Math.toRadians(obj.roll || 0)
    },
    duration: obj.duration === undefined ? 3 : obj.duration,
    complete: obj.complete
  });
};

/* eslint-disable no-unused-vars */
var Cesium$7 = window.Cesium;
var defined$3 = Cesium$7.defined,
    Ray = Cesium$7.Ray,
    Cartesian3 = Cesium$7.Cartesian3,
    Cartographic$1 = Cesium$7.Cartographic,
    ReferenceFrame = Cesium$7.ReferenceFrame,
    SceneMode = Cesium$7.SceneMode;
var Utils = {};
var unprojectedScratch = new Cartographic$1();
var rayScratch = new Ray();
/**
 * gets the focus point of the camera
 * @param {Viewer|Widget} terria The terria
 * @param {boolean} inWorldCoordinates true to get the focus in world coordinates, otherwise get it in projection-specific map coordinates, in meters.
 * @param {Cartesian3} [result] The object in which the result will be stored.
 * @return {Cartesian3} The modified result parameter, a new instance if none was provided or undefined if there is no focus point.
 */

Utils.getCameraFocus = function (terria, inWorldCoordinates, result) {
  var scene = terria.scene;
  var camera = scene.camera;

  if (scene.mode === SceneMode.MORPHING) {
    return undefined;
  }

  if (!defined$3(result)) {
    result = new Cartesian3();
  } // TODO bug when tracking: if entity moves the current position should be used and not only the one when starting orbiting/rotating
  // TODO bug when tracking: reset should reset to default view of tracked entity


  if (defined$3(terria.trackedEntity)) {
    result = terria.trackedEntity.position.getValue(terria.clock.currentTime, result);
  } else {
    rayScratch.origin = camera.positionWC;
    rayScratch.direction = camera.directionWC;
    result = scene.globe.pick(rayScratch, scene, result);
  }

  if (!defined$3(result)) {
    return undefined;
  }

  if (scene.mode === SceneMode.SCENE2D || scene.mode === SceneMode.COLUMBUS_VIEW) {
    result = camera.worldToCameraCoordinatesPoint(result, result);

    if (inWorldCoordinates) {
      result = scene.globe.ellipsoid.cartographicToCartesian(scene.mapProjection.unproject(result, unprojectedScratch), result);
    }
  } else {
    if (!inWorldCoordinates) {
      result = camera.worldToCameraCoordinatesPoint(result, result);
    }
  }

  return result;
};

var Cesium$8 = window.Cesium;
var defined$4 = Cesium$8.defined,
    Ray$1 = Cesium$8.Ray,
    IntersectionTests = Cesium$8.IntersectionTests,
    Cartesian3$1 = Cesium$8.Cartesian3,
    SceneMode$1 = Cesium$8.SceneMode;
/**
 * The model for a zoom in control in the navigation control tool bar
 *
 * @alias ZoomOutNavigationControl
 * @constructor
 * @abstract
 *
 * @param {Terria} terria The Terria instance.
 * @param {boolean} zoomIn is used for zooming in (true) or out (false)
 */

var ZoomNavigationControl = function ZoomNavigationControl(terria, zoomIn) {
  NavigationControl.apply(this, arguments);
  /**
   * Gets or sets the name of the control which is set as the control's title.
   * This property is observable.
   * @type {String}
   */

  this.name = '视图 ' + (zoomIn ? '放大' : '缩小');
  /**
   * Gets or sets the text to be displayed in the nav control. Controls that
   * have text do not display the svgIcon.
   * This property is observable.
   * @type {String}
   */

  this.text = zoomIn ? '+' : '-';
  /**
   * Gets or sets the CSS class of the control. This property is observable.
   * @type {String}
   */

  this.cssClass = 'navigation-control-icon-zoom-' + (zoomIn ? 'in' : 'out');
  this.relativeAmount = 2;

  if (zoomIn) {
    // this ensures that zooming in is the inverse of zooming out and vice versa
    // e.g. the camera position remains when zooming in and out
    this.relativeAmount = 1 / this.relativeAmount;
  }
};

ZoomNavigationControl.prototype.relativeAmount = 1;
ZoomNavigationControl.prototype = Object.create(NavigationControl.prototype);
/**
 * When implemented in a derived class, performs an action when the user clicks
 * on this control
 * @abstract
 * @protected
 */

ZoomNavigationControl.prototype.activate = function () {
  this.zoom(this.relativeAmount);
};

var cartesian3Scratch = new Cartesian3$1();

ZoomNavigationControl.prototype.zoom = function (relativeAmount) {
  // this.terria.analytics.logEvent('navigation', 'click', 'zoomIn');
  this.isActive = true;

  if (defined$4(this.terria)) {
    var scene = this.terria.scene;
    var sscc = scene.screenSpaceCameraController; // do not zoom if it is disabled

    if (!sscc.enableInputs || !sscc.enableZoom) {
      return;
    } // TODO
    //            if(scene.mode == SceneMode.COLUMBUS_VIEW && !sscc.enableTranslate) {
    //                return;
    //            }


    var camera = scene.camera;
    var orientation;

    switch (scene.mode) {
      case SceneMode$1.MORPHING:
        break;

      case SceneMode$1.SCENE2D:
        camera.zoomIn(camera.positionCartographic.height * (1 - this.relativeAmount));
        break;

      default:
        var focus;

        if (defined$4(this.terria.trackedEntity)) {
          focus = new Cartesian3$1();
        } else {
          focus = Utils.getCameraFocus(this.terria, false);
        }

        if (!defined$4(focus)) {
          // Camera direction is not pointing at the globe, so use the ellipsoid horizon point as
          // the focal point.
          var ray = new Ray$1(camera.worldToCameraCoordinatesPoint(scene.globe.ellipsoid.cartographicToCartesian(camera.positionCartographic)), camera.directionWC);
          focus = IntersectionTests.grazingAltitudeLocation(ray, scene.globe.ellipsoid);
          orientation = {
            heading: camera.heading,
            pitch: camera.pitch,
            roll: camera.roll
          };
        } else {
          orientation = {
            direction: camera.direction,
            up: camera.up
          };
        }

        var direction = Cartesian3$1.subtract(camera.position, focus, cartesian3Scratch);
        var movementVector = Cartesian3$1.multiplyByScalar(direction, relativeAmount, direction);
        var endPosition = Cartesian3$1.add(focus, movementVector, focus);

        if (defined$4(this.terria.trackedEntity) || scene.mode === SceneMode$1.COLUMBUS_VIEW) {
          // sometimes flyTo does not work (jumps to wrong position) so just set the position without any animation
          // do not use flyTo when tracking an entity because during animatiuon the position of the entity may change
          camera.position = endPosition;
        } else {
          camera.flyTo({
            destination: endPosition,
            orientation: orientation,
            duration: 0.5,
            convert: false
          });
        }

    }
  } // this.terria.notifyRepaintRequired();


  this.isActive = false;
};

var svgCompassOuterRing = 'm 66.5625,0 0,15.15625 3.71875,0 0,-10.40625 5.5,10.40625 4.375,0 0,-15.15625 -3.71875,0 0,10.40625 L 70.9375,0 66.5625,0 z M 72.5,20.21875 c -28.867432,0 -52.28125,23.407738 -52.28125,52.28125 0,28.87351 23.413818,52.3125 52.28125,52.3125 28.86743,0 52.28125,-23.43899 52.28125,-52.3125 0,-28.873512 -23.41382,-52.28125 -52.28125,-52.28125 z m 0,1.75 c 13.842515,0 26.368948,5.558092 35.5,14.5625 l -11.03125,11 0.625,0.625 11.03125,-11 c 8.9199,9.108762 14.4375,21.579143 14.4375,35.34375 0,13.764606 -5.5176,26.22729 -14.4375,35.34375 l -11.03125,-11 -0.625,0.625 11.03125,11 c -9.130866,9.01087 -21.658601,14.59375 -35.5,14.59375 -13.801622,0 -26.321058,-5.53481 -35.4375,-14.5 l 11.125,-11.09375 c 6.277989,6.12179 14.857796,9.90625 24.3125,9.90625 19.241896,0 34.875,-15.629154 34.875,-34.875 0,-19.245847 -15.633104,-34.84375 -34.875,-34.84375 -9.454704,0 -18.034511,3.760884 -24.3125,9.875 L 37.0625,36.4375 C 46.179178,27.478444 58.696991,21.96875 72.5,21.96875 z m -0.875,0.84375 0,13.9375 1.75,0 0,-13.9375 -1.75,0 z M 36.46875,37.0625 47.5625,48.15625 C 41.429794,54.436565 37.65625,63.027539 37.65625,72.5 c 0,9.472461 3.773544,18.055746 9.90625,24.34375 L 36.46875,107.9375 c -8.96721,-9.1247 -14.5,-21.624886 -14.5,-35.4375 0,-13.812615 5.53279,-26.320526 14.5,-35.4375 z M 72.5,39.40625 c 18.297686,0 33.125,14.791695 33.125,33.09375 0,18.302054 -14.827314,33.125 -33.125,33.125 -18.297687,0 -33.09375,-14.822946 -33.09375,-33.125 0,-18.302056 14.796063,-33.09375 33.09375,-33.09375 z M 22.84375,71.625 l 0,1.75 13.96875,0 0,-1.75 -13.96875,0 z m 85.5625,0 0,1.75 14,0 0,-1.75 -14,0 z M 71.75,108.25 l 0,13.9375 1.71875,0 0,-13.9375 -1.71875,0 z';

var svgCompassGyro = 'm 72.71875,54.375 c -0.476702,0 -0.908208,0.245402 -1.21875,0.5625 -0.310542,0.317098 -0.551189,0.701933 -0.78125,1.1875 -0.172018,0.363062 -0.319101,0.791709 -0.46875,1.25 -6.91615,1.075544 -12.313231,6.656514 -13,13.625 -0.327516,0.117495 -0.661877,0.244642 -0.9375,0.375 -0.485434,0.22959 -0.901634,0.471239 -1.21875,0.78125 -0.317116,0.310011 -0.5625,0.742111 -0.5625,1.21875 l 0.03125,0 c 0,0.476639 0.245384,0.877489 0.5625,1.1875 0.317116,0.310011 0.702066,0.58291 1.1875,0.8125 0.35554,0.168155 0.771616,0.32165 1.21875,0.46875 1.370803,6.10004 6.420817,10.834127 12.71875,11.8125 0.146999,0.447079 0.30025,0.863113 0.46875,1.21875 0.230061,0.485567 0.470708,0.870402 0.78125,1.1875 0.310542,0.317098 0.742048,0.5625 1.21875,0.5625 0.476702,0 0.876958,-0.245402 1.1875,-0.5625 0.310542,-0.317098 0.582439,-0.701933 0.8125,-1.1875 0.172018,-0.363062 0.319101,-0.791709 0.46875,-1.25 6.249045,-1.017063 11.256351,-5.7184 12.625,-11.78125 0.447134,-0.1471 0.86321,-0.300595 1.21875,-0.46875 0.485434,-0.22959 0.901633,-0.502489 1.21875,-0.8125 0.317117,-0.310011 0.5625,-0.710861 0.5625,-1.1875 l -0.03125,0 c 0,-0.476639 -0.245383,-0.908739 -0.5625,-1.21875 C 89.901633,71.846239 89.516684,71.60459 89.03125,71.375 88.755626,71.244642 88.456123,71.117495 88.125,71 87.439949,64.078341 82.072807,58.503735 75.21875,57.375 c -0.15044,-0.461669 -0.326927,-0.884711 -0.5,-1.25 -0.230061,-0.485567 -0.501958,-0.870402 -0.8125,-1.1875 -0.310542,-0.317098 -0.710798,-0.5625 -1.1875,-0.5625 z m -0.0625,1.40625 c 0.03595,-0.01283 0.05968,0 0.0625,0 0.0056,0 0.04321,-0.02233 0.1875,0.125 0.144288,0.147334 0.34336,0.447188 0.53125,0.84375 0.06385,0.134761 0.123901,0.309578 0.1875,0.46875 -0.320353,-0.01957 -0.643524,-0.0625 -0.96875,-0.0625 -0.289073,0 -0.558569,0.04702 -0.84375,0.0625 C 71.8761,57.059578 71.936151,56.884761 72,56.75 c 0.18789,-0.396562 0.355712,-0.696416 0.5,-0.84375 0.07214,-0.07367 0.120304,-0.112167 0.15625,-0.125 z m 0,2.40625 c 0.448007,0 0.906196,0.05436 1.34375,0.09375 0.177011,0.592256 0.347655,1.271044 0.5,2.03125 0.475097,2.370753 0.807525,5.463852 0.9375,8.9375 -0.906869,-0.02852 -1.834463,-0.0625 -2.78125,-0.0625 -0.92298,0 -1.802327,0.03537 -2.6875,0.0625 0.138529,-3.473648 0.493653,-6.566747 0.96875,-8.9375 0.154684,-0.771878 0.320019,-1.463985 0.5,-2.0625 0.405568,-0.03377 0.804291,-0.0625 1.21875,-0.0625 z m -2.71875,0.28125 c -0.129732,0.498888 -0.259782,0.987558 -0.375,1.5625 -0.498513,2.487595 -0.838088,5.693299 -0.96875,9.25 -3.21363,0.15162 -6.119596,0.480068 -8.40625,0.9375 -0.682394,0.136509 -1.275579,0.279657 -1.84375,0.4375 0.799068,-6.135482 5.504716,-11.036454 11.59375,-12.1875 z M 75.5,58.5 c 6.043169,1.18408 10.705093,6.052712 11.5,12.15625 -0.569435,-0.155806 -1.200273,-0.302525 -1.875,-0.4375 -2.262525,-0.452605 -5.108535,-0.783809 -8.28125,-0.9375 -0.130662,-3.556701 -0.470237,-6.762405 -0.96875,-9.25 C 75.761959,59.467174 75.626981,58.990925 75.5,58.5 z m -2.84375,12.09375 c 0.959338,0 1.895843,0.03282 2.8125,0.0625 C 75.48165,71.267751 75.5,71.871028 75.5,72.5 c 0,1.228616 -0.01449,2.438313 -0.0625,3.59375 -0.897358,0.0284 -1.811972,0.0625 -2.75,0.0625 -0.927373,0 -1.831062,-0.03473 -2.71875,-0.0625 -0.05109,-1.155437 -0.0625,-2.365134 -0.0625,-3.59375 0,-0.628972 0.01741,-1.232249 0.03125,-1.84375 0.895269,-0.02827 1.783025,-0.0625 2.71875,-0.0625 z M 68.5625,70.6875 c -0.01243,0.60601 -0.03125,1.189946 -0.03125,1.8125 0,1.22431 0.01541,2.407837 0.0625,3.5625 -3.125243,-0.150329 -5.92077,-0.471558 -8.09375,-0.90625 -0.784983,-0.157031 -1.511491,-0.316471 -2.125,-0.5 -0.107878,-0.704096 -0.1875,-1.422089 -0.1875,-2.15625 0,-0.115714 0.02849,-0.228688 0.03125,-0.34375 0.643106,-0.20284 1.389577,-0.390377 2.25,-0.5625 2.166953,-0.433487 4.97905,-0.75541 8.09375,-0.90625 z m 8.3125,0.03125 c 3.075121,0.15271 5.824455,0.446046 7.96875,0.875 0.857478,0.171534 1.630962,0.360416 2.28125,0.5625 0.0027,0.114659 0,0.228443 0,0.34375 0,0.735827 -0.07914,1.450633 -0.1875,2.15625 -0.598568,0.180148 -1.29077,0.34562 -2.0625,0.5 -2.158064,0.431708 -4.932088,0.754666 -8.03125,0.90625 0.04709,-1.154663 0.0625,-2.33819 0.0625,-3.5625 0,-0.611824 -0.01924,-1.185379 -0.03125,-1.78125 z M 57.15625,72.5625 c 0.0023,0.572772 0.06082,1.131112 0.125,1.6875 -0.125327,-0.05123 -0.266577,-0.10497 -0.375,-0.15625 -0.396499,-0.187528 -0.665288,-0.387337 -0.8125,-0.53125 -0.147212,-0.143913 -0.15625,-0.182756 -0.15625,-0.1875 0,-0.0047 -0.02221,-0.07484 0.125,-0.21875 0.147212,-0.143913 0.447251,-0.312472 0.84375,-0.5 0.07123,-0.03369 0.171867,-0.06006 0.25,-0.09375 z m 31.03125,0 c 0.08201,0.03503 0.175941,0.05872 0.25,0.09375 0.396499,0.187528 0.665288,0.356087 0.8125,0.5 0.14725,0.14391 0.15625,0.21405 0.15625,0.21875 0,0.0047 -0.009,0.04359 -0.15625,0.1875 -0.147212,0.143913 -0.416001,0.343722 -0.8125,0.53125 -0.09755,0.04613 -0.233314,0.07889 -0.34375,0.125 0.06214,-0.546289 0.09144,-1.094215 0.09375,-1.65625 z m -29.5,3.625 c 0.479308,0.123125 0.983064,0.234089 1.53125,0.34375 2.301781,0.460458 5.229421,0.787224 8.46875,0.9375 0.167006,2.84339 0.46081,5.433176 0.875,7.5 0.115218,0.574942 0.245268,1.063612 0.375,1.5625 -5.463677,-1.028179 -9.833074,-5.091831 -11.25,-10.34375 z m 27.96875,0 C 85.247546,81.408945 80.919274,85.442932 75.5,86.5 c 0.126981,-0.490925 0.261959,-0.967174 0.375,-1.53125 0.41419,-2.066824 0.707994,-4.65661 0.875,-7.5 3.204493,-0.15162 6.088346,-0.480068 8.375,-0.9375 0.548186,-0.109661 1.051942,-0.220625 1.53125,-0.34375 z M 70.0625,77.53125 c 0.865391,0.02589 1.723666,0.03125 2.625,0.03125 0.912062,0 1.782843,-0.0048 2.65625,-0.03125 -0.165173,2.736408 -0.453252,5.207651 -0.84375,7.15625 -0.152345,0.760206 -0.322989,1.438994 -0.5,2.03125 -0.437447,0.03919 -0.895856,0.0625 -1.34375,0.0625 -0.414943,0 -0.812719,-0.02881 -1.21875,-0.0625 -0.177011,-0.592256 -0.347655,-1.271044 -0.5,-2.03125 -0.390498,-1.948599 -0.700644,-4.419842 -0.875,-7.15625 z m 1.75,10.28125 c 0.284911,0.01545 0.554954,0.03125 0.84375,0.03125 0.325029,0 0.648588,-0.01171 0.96875,-0.03125 -0.05999,0.148763 -0.127309,0.31046 -0.1875,0.4375 -0.18789,0.396562 -0.386962,0.696416 -0.53125,0.84375 -0.144288,0.147334 -0.181857,0.125 -0.1875,0.125 -0.0056,0 -0.07446,0.02233 -0.21875,-0.125 C 72.355712,88.946416 72.18789,88.646562 72,88.25 71.939809,88.12296 71.872486,87.961263 71.8125,87.8125 z';

var svgCompassRotationMarker = 'M 72.46875,22.03125 C 59.505873,22.050338 46.521615,27.004287 36.6875,36.875 L 47.84375,47.96875 C 61.521556,34.240041 83.442603,34.227389 97.125,47.90625 l 11.125,-11.125 C 98.401629,26.935424 85.431627,22.012162 72.46875,22.03125 z';

var Cesium$9 = window.Cesium;
var defined$5 = Cesium$9.defined,
    getTimestamp$1 = Cesium$9.getTimestamp,
    EventHelper$1 = Cesium$9.EventHelper,
    Transforms = Cesium$9.Transforms,
    SceneMode$2 = Cesium$9.SceneMode,
    Cartesian2$1 = Cesium$9.Cartesian2,
    Cartesian3$2 = Cesium$9.Cartesian3,
    Matrix4 = Cesium$9.Matrix4,
    BoundingSphere = Cesium$9.BoundingSphere,
    HeadingPitchRange = Cesium$9.HeadingPitchRange,
    knockout$5 = Cesium$9.knockout;
var cesiumMath = Cesium$9.Math;

var Knockout$4 = knockout$5;

var NavigationViewModel = function NavigationViewModel(options) {
  this.terria = options.terria;
  this.eventHelper = new EventHelper$1();
  this.enableZoomControls = defined$5(options.enableZoomControls) ? options.enableZoomControls : true;
  this.enableCompass = defined$5(options.enableCompass) ? options.enableCompass : true;
  this.navigationLocked = false; // if (this.showZoomControls)
  //   {

  this.controls = options.controls;

  if (!defined$5(this.controls)) {
    this.controls = [new ZoomNavigationControl(this.terria, true), new ResetViewNavigationControl(this.terria), new ZoomNavigationControl(this.terria, false)];
  } // }


  this.svgCompassOuterRing = svgCompassOuterRing;
  this.svgCompassGyro = svgCompassGyro;
  this.svgCompassRotationMarker = svgCompassRotationMarker;
  this.showCompass = defined$5(this.terria) && this.enableCompass;
  this.heading = this.showCompass ? this.terria.scene.camera.heading : 0.0;
  this.isOrbiting = false;
  this.orbitCursorAngle = 0;
  this.orbitCursorOpacity = 0.0;
  this.orbitLastTimestamp = 0;
  this.orbitFrame = undefined;
  this.orbitIsLook = false;
  this.orbitMouseMoveFunction = undefined;
  this.orbitMouseUpFunction = undefined;
  this.isRotating = false;
  this.rotateInitialCursorAngle = undefined;
  this.rotateFrame = undefined;
  this.rotateIsLook = false;
  this.rotateMouseMoveFunction = undefined;
  this.rotateMouseUpFunction = undefined;
  this._unsubcribeFromPostRender = undefined;
  Knockout$4.track(this, ['controls', 'showCompass', 'heading', 'isOrbiting', 'orbitCursorAngle', 'isRotating']);
  var that = this;

  NavigationViewModel.prototype.setNavigationLocked = function (locked) {
    this.navigationLocked = locked;

    if (this.controls && this.controls.length > 1) {
      this.controls[1].setNavigationLocked(this.navigationLocked);
    }
  };

  function widgetChange() {
    if (defined$5(that.terria)) {
      if (that._unsubcribeFromPostRender) {
        that._unsubcribeFromPostRender();

        that._unsubcribeFromPostRender = undefined;
      }

      that.showCompass =  that.enableCompass;
      that._unsubcribeFromPostRender = that.terria.scene.postRender.addEventListener(function () {
        that.heading = that.terria.scene.camera.heading;
      });
    } else {
      if (that._unsubcribeFromPostRender) {
        that._unsubcribeFromPostRender();

        that._unsubcribeFromPostRender = undefined;
      }

      that.showCompass = false;
    }
  }

  this.eventHelper.add(this.terria.afterWidgetChanged, widgetChange, this); // this.terria.afterWidgetChanged.addEventListener(widgetChange);

  widgetChange();
};

NavigationViewModel.prototype.destroy = function () {
  this.eventHelper.removeAll(); // loadView(require('fs').readFileSync(baseURLEmpCesium + 'js-lib/terrajs/lib/Views/Navigation.html', 'utf8'), container, this);
};

NavigationViewModel.prototype.show = function (container) {
  var compassDisplay = this.enableCompass == undefined ? true : this.enableCompass;
  compassDisplay = compassDisplay ? "block" : "none";
  var enableZoomControlsDisplay = this.enableZoomControls == undefined ? true : this.enableZoomControls;
  enableZoomControlsDisplay = enableZoomControlsDisplay ? "block" : "none";
  var testing = '<div class="compass" id="vis3d-compass" title="" style="display:' + compassDisplay + '" data-bind="visible: showCompass, event: { mousedown: handleMouseDown, dblclick: handleDoubleClick }">' + '<div class="compass-outer-ring-background"></div>' + ' <div class="compass-rotation-marker" data-bind="visible: isOrbiting, style: { transform: \'rotate(-\' + orbitCursorAngle + \'rad)\', \'-webkit-transform\': \'rotate(-\' + orbitCursorAngle + \'rad)\', opacity: orbitCursorOpacity }, cesiumSvgPath: { path: svgCompassRotationMarker, width: 145, height: 145 }"></div>' + ' <div class="compass-outer-ring" title="" data-bind="style: { transform: \'rotate(-\' + heading + \'rad)\', \'-webkit-transform\': \'rotate(-\' + heading + \'rad)\' }, cesiumSvgPath: { path: svgCompassOuterRing, width: 145, height: 145 }"></div>' + ' <div class="compass-gyro-background"></div>' + ' <div class="compass-gyro" data-bind="cesiumSvgPath: { path: svgCompassGyro, width: 145, height: 145 }, css: { \'compass-gyro-active\': isOrbiting }"></div>' + '</div>' + '<div class="navigation-controls" id="vis3d-navigation-controls" style="display:' + enableZoomControlsDisplay + '">' + '<!-- ko foreach: controls -->' + '<div data-bind="click: activate, attr: { title: $data.name }, css: $root.isLastControl($data) ? \'navigation-control-last\' : \'navigation-control\' ">' + '   <!-- ko if: $data.hasText -->' + '   <div data-bind="text: $data.text, css: $data.isActive ?  \'navigation-control-icon-active \' + $data.cssClass : $data.cssClass"></div>' + '   <!-- /ko -->' + '  <!-- ko ifnot: $data.hasText -->' + '  <div data-bind="cesiumSvgPath: { path: $data.svgIcon, width: $data.svgWidth, height: $data.svgHeight }, css: $data.isActive ?  \'navigation-control-icon-active \' + $data.cssClass : $data.cssClass"></div>' + '  <!-- /ko -->' + ' </div>' + ' <!-- /ko -->' + '</div>';
  loadView(testing, container, this); // loadView(navigatorTemplate, container, this);
  // loadView(require('fs').readFileSync(baseURLEmpCesium + 'js-lib/terrajs/lib/Views/Navigation.html', 'utf8'), container, this);
};

NavigationViewModel.prototype.setStyle = function (style) {
  if (!style || Object.keys(style).length < 1) return;
  var ele = document.getElementById("vis3d-compass");
  var ele2 = document.getElementById("vis3d-navigation-controls");
  if (!ele) return;

  for (var i in style) {
    ele.style[i] = style[i];
    ele2.style[i] = style[i];

    if (i == "top") {
      ele2.style[i] = parseFloat(style[i]) + 100 + "px";
    }

    if (i == "bottom") {
      ele.style[i] = parseFloat(style[i]) + 82 + "px";
    }

    if (i == "left") {
      ele2.style.left = parseFloat(ele.style.left) + 30 + "px";
    }

    if (i == "right") {
      ele2.style.right = parseFloat(ele.style.right) - 30 + "px";
    }
  }
};
/**
 * Adds a control to this toolbar.
 * @param {NavControl} control The control to add.
 */


NavigationViewModel.prototype.add = function (control) {
  this.controls.push(control);
};
/**
 * Removes a control from this toolbar.
 * @param {NavControl} control The control to remove.
 */


NavigationViewModel.prototype.remove = function (control) {
  this.controls.remove(control);
};
/**
 * Checks if the control given is the last control in the control array.
 * @param {NavControl} control The control to remove.
 */


NavigationViewModel.prototype.isLastControl = function (control) {
  return control === this.controls[this.controls.length - 1];
};

var vectorScratch = new Cartesian2$1();

NavigationViewModel.prototype.handleMouseDown = function (viewModel, e) {
  var scene = this.terria.scene;

  if (scene.mode === SceneMode$2.MORPHING) {
    return true;
  }

  if (viewModel.navigationLocked) {
    return true;
  }

  var compassElement = e.currentTarget;
  var compassRectangle = e.currentTarget.getBoundingClientRect();
  var maxDistance = compassRectangle.width / 2.0;
  var center = new Cartesian2$1((compassRectangle.right - compassRectangle.left) / 2.0, (compassRectangle.bottom - compassRectangle.top) / 2.0);
  var clickLocation = new Cartesian2$1(e.clientX - compassRectangle.left, e.clientY - compassRectangle.top);
  var vector = Cartesian2$1.subtract(clickLocation, center, vectorScratch);
  var distanceFromCenter = Cartesian2$1.magnitude(vector);
  var distanceFraction = distanceFromCenter / maxDistance;
  var nominalTotalRadius = 145;
  var norminalGyroRadius = 50;

  if (distanceFraction < norminalGyroRadius / nominalTotalRadius) {
    orbit(this, compassElement, vector); //            return false;
  } else if (distanceFraction < 1.0) {
    rotate(this, compassElement, vector); //            return false;
  } else {
    return true;
  }
};

var oldTransformScratch = new Matrix4();
var newTransformScratch = new Matrix4();
var centerScratch = new Cartesian3$2();

NavigationViewModel.prototype.handleDoubleClick = function (viewModel, e) {
  var scene = viewModel.terria.scene;
  var camera = scene.camera;
  var sscc = scene.screenSpaceCameraController;

  if (scene.mode === SceneMode$2.MORPHING || !sscc.enableInputs) {
    return true;
  }

  if (viewModel.navigationLocked) {
    return true;
  }

  if (scene.mode === SceneMode$2.COLUMBUS_VIEW && !sscc.enableTranslate) {
    return;
  }

  if (scene.mode === SceneMode$2.SCENE3D || scene.mode === SceneMode$2.COLUMBUS_VIEW) {
    if (!sscc.enableLook) {
      return;
    }

    if (scene.mode === SceneMode$2.SCENE3D) {
      if (!sscc.enableRotate) {
        return;
      }
    }
  }

  var center = Utils.getCameraFocus(viewModel.terria, true, centerScratch);

  if (!defined$5(center)) {
    // Globe is barely visible, so reset to home view.
    this.controls[1].resetView();
    return;
  }

  var cameraPosition = scene.globe.ellipsoid.cartographicToCartesian(camera.positionCartographic, new Cartesian3$2());
  var surfaceNormal = scene.globe.ellipsoid.geodeticSurfaceNormal(center);
  var focusBoundingSphere = new BoundingSphere(center, 0);
  camera.flyToBoundingSphere(focusBoundingSphere, {
    offset: new HeadingPitchRange(0, // do not use camera.pitch since the pitch at the center/target is required
    cesiumMath.PI_OVER_TWO - Cartesian3$2.angleBetween(surfaceNormal, camera.directionWC), // distanceToBoundingSphere returns wrong values when in 2D or Columbus view so do not use
    // camera.distanceToBoundingSphere(focusBoundingSphere)
    // instead calculate distance manually
    Cartesian3$2.distance(cameraPosition, center)),
    duration: 1.5
  });
};

NavigationViewModel.create = function (options) {
  // options.enableZoomControls = this.enableZoomControls;
  // options.enableCompass = this.enableCompass;
  var result = new NavigationViewModel(options);
  result.show(options.container);
  result.setStyle(options.style);
  return result;
};

function orbit(viewModel, compassElement, cursorVector) {
  var scene = viewModel.terria.scene;
  var sscc = scene.screenSpaceCameraController; // do not orbit if it is disabled

  if (scene.mode === SceneMode$2.MORPHING || !sscc.enableInputs) {
    return;
  }

  if (viewModel.navigationLocked) {
    return true;
  }

  switch (scene.mode) {
    case SceneMode$2.COLUMBUS_VIEW:
      if (sscc.enableLook) {
        break;
      }

      if (!sscc.enableTranslate || !sscc.enableTilt) {
        return;
      }

      break;

    case SceneMode$2.SCENE3D:
      if (sscc.enableLook) {
        break;
      }

      if (!sscc.enableTilt || !sscc.enableRotate) {
        return;
      }

      break;

    case SceneMode$2.SCENE2D:
      if (!sscc.enableTranslate) {
        return;
      }

      break;
  } // Remove existing event handlers, if any.


  document.removeEventListener('mousemove', viewModel.orbitMouseMoveFunction, false);
  document.removeEventListener('mouseup', viewModel.orbitMouseUpFunction, false);

  if (defined$5(viewModel.orbitTickFunction)) {
    viewModel.terria.clock.onTick.removeEventListener(viewModel.orbitTickFunction);
  }

  viewModel.orbitMouseMoveFunction = undefined;
  viewModel.orbitMouseUpFunction = undefined;
  viewModel.orbitTickFunction = undefined;
  viewModel.isOrbiting = true;
  viewModel.orbitLastTimestamp = getTimestamp$1();
  var camera = scene.camera;

  if (defined$5(viewModel.terria.trackedEntity)) {
    // when tracking an entity simply use that reference frame
    viewModel.orbitFrame = undefined;
    viewModel.orbitIsLook = false;
  } else {
    var center = Utils.getCameraFocus(viewModel.terria, true, centerScratch);

    if (!defined$5(center)) {
      viewModel.orbitFrame = Transforms.eastNorthUpToFixedFrame(camera.positionWC, scene.globe.ellipsoid, newTransformScratch);
      viewModel.orbitIsLook = true;
    } else {
      viewModel.orbitFrame = Transforms.eastNorthUpToFixedFrame(center, scene.globe.ellipsoid, newTransformScratch);
      viewModel.orbitIsLook = false;
    }
  }

  viewModel.orbitTickFunction = function (e) {
    var timestamp = getTimestamp$1();
    var deltaT = timestamp - viewModel.orbitLastTimestamp;
    var rate = (viewModel.orbitCursorOpacity - 0.5) * 2.5 / 1000;
    var distance = deltaT * rate;
    var angle = viewModel.orbitCursorAngle + cesiumMath.PI_OVER_TWO;
    var x = Math.cos(angle) * distance;
    var y = Math.sin(angle) * distance;
    var oldTransform;

    if (viewModel.navigationLocked) {
      return true;
    }

    if (defined$5(viewModel.orbitFrame)) {
      oldTransform = Matrix4.clone(camera.transform, oldTransformScratch);
      camera.lookAtTransform(viewModel.orbitFrame);
    } // do not look up/down or rotate in 2D mode


    if (scene.mode === SceneMode$2.SCENE2D) {
      camera.move(new Cartesian3$2(x, y, 0), Math.max(scene.canvas.clientWidth, scene.canvas.clientHeight) / 100 * camera.positionCartographic.height * distance);
    } else {
      if (viewModel.orbitIsLook) {
        camera.look(Cartesian3$2.UNIT_Z, -x);
        camera.look(camera.right, -y);
      } else {
        camera.rotateLeft(x);
        camera.rotateUp(y);
      }
    }

    if (defined$5(viewModel.orbitFrame)) {
      camera.lookAtTransform(oldTransform);
    } // viewModel.terria.cesium.notifyRepaintRequired();


    viewModel.orbitLastTimestamp = timestamp;
  };

  function updateAngleAndOpacity(vector, compassWidth) {
    var angle = Math.atan2(-vector.y, vector.x);
    viewModel.orbitCursorAngle = cesiumMath.zeroToTwoPi(angle - cesiumMath.PI_OVER_TWO);
    var distance = Cartesian2$1.magnitude(vector);
    var maxDistance = compassWidth / 2.0;
    var distanceFraction = Math.min(distance / maxDistance, 1.0);
    var easedOpacity = 0.5 * distanceFraction * distanceFraction + 0.5;
    viewModel.orbitCursorOpacity = easedOpacity; // viewModel.terria.cesium.notifyRepaintRequired();
  }

  viewModel.orbitMouseMoveFunction = function (e) {
    var compassRectangle = compassElement.getBoundingClientRect();
    var center = new Cartesian2$1((compassRectangle.right - compassRectangle.left) / 2.0, (compassRectangle.bottom - compassRectangle.top) / 2.0);
    var clickLocation = new Cartesian2$1(e.clientX - compassRectangle.left, e.clientY - compassRectangle.top);
    var vector = Cartesian2$1.subtract(clickLocation, center, vectorScratch);
    updateAngleAndOpacity(vector, compassRectangle.width);
  };

  viewModel.orbitMouseUpFunction = function (e) {
    // TODO: if mouse didn't move, reset view to looking down, north is up?
    viewModel.isOrbiting = false;
    document.removeEventListener('mousemove', viewModel.orbitMouseMoveFunction, false);
    document.removeEventListener('mouseup', viewModel.orbitMouseUpFunction, false);

    if (defined$5(viewModel.orbitTickFunction)) {
      viewModel.terria.clock.onTick.removeEventListener(viewModel.orbitTickFunction);
    }

    viewModel.orbitMouseMoveFunction = undefined;
    viewModel.orbitMouseUpFunction = undefined;
    viewModel.orbitTickFunction = undefined;
  };

  document.addEventListener('mousemove', viewModel.orbitMouseMoveFunction, false);
  document.addEventListener('mouseup', viewModel.orbitMouseUpFunction, false);
  viewModel.terria.clock.onTick.addEventListener(viewModel.orbitTickFunction);
  updateAngleAndOpacity(cursorVector, compassElement.getBoundingClientRect().width);
}

function rotate(viewModel, compassElement, cursorVector) {
  var scene = viewModel.terria.scene;
  var camera = scene.camera;
  var sscc = scene.screenSpaceCameraController; // do not rotate in 2D mode or if rotating is disabled

  if (scene.mode === SceneMode$2.MORPHING || scene.mode === SceneMode$2.SCENE2D || !sscc.enableInputs) {
    return;
  }

  if (viewModel.navigationLocked) {
    return true;
  }

  if (!sscc.enableLook && (scene.mode === SceneMode$2.COLUMBUS_VIEW || scene.mode === SceneMode$2.SCENE3D && !sscc.enableRotate)) {
    return;
  } // Remove existing event handlers, if any.


  document.removeEventListener('mousemove', viewModel.rotateMouseMoveFunction, false);
  document.removeEventListener('mouseup', viewModel.rotateMouseUpFunction, false);
  viewModel.rotateMouseMoveFunction = undefined;
  viewModel.rotateMouseUpFunction = undefined;
  viewModel.isRotating = true;
  viewModel.rotateInitialCursorAngle = Math.atan2(-cursorVector.y, cursorVector.x);

  if (defined$5(viewModel.terria.trackedEntity)) {
    // when tracking an entity simply use that reference frame
    viewModel.rotateFrame = undefined;
    viewModel.rotateIsLook = false;
  } else {
    var viewCenter = Utils.getCameraFocus(viewModel.terria, true, centerScratch);

    if (!defined$5(viewCenter) || scene.mode === SceneMode$2.COLUMBUS_VIEW && !sscc.enableLook && !sscc.enableTranslate) {
      viewModel.rotateFrame = Transforms.eastNorthUpToFixedFrame(camera.positionWC, scene.globe.ellipsoid, newTransformScratch);
      viewModel.rotateIsLook = true;
    } else {
      viewModel.rotateFrame = Transforms.eastNorthUpToFixedFrame(viewCenter, scene.globe.ellipsoid, newTransformScratch);
      viewModel.rotateIsLook = false;
    }
  }

  var oldTransform;

  if (defined$5(viewModel.rotateFrame)) {
    oldTransform = Matrix4.clone(camera.transform, oldTransformScratch);
    camera.lookAtTransform(viewModel.rotateFrame);
  }

  viewModel.rotateInitialCameraAngle = -camera.heading;

  if (defined$5(viewModel.rotateFrame)) {
    camera.lookAtTransform(oldTransform);
  }

  viewModel.rotateMouseMoveFunction = function (e) {
    var compassRectangle = compassElement.getBoundingClientRect();
    var center = new Cartesian2$1((compassRectangle.right - compassRectangle.left) / 2.0, (compassRectangle.bottom - compassRectangle.top) / 2.0);
    var clickLocation = new Cartesian2$1(e.clientX - compassRectangle.left, e.clientY - compassRectangle.top);
    var vector = Cartesian2$1.subtract(clickLocation, center, vectorScratch);
    var angle = Math.atan2(-vector.y, vector.x);
    var angleDifference = angle - viewModel.rotateInitialCursorAngle;
    var newCameraAngle = cesiumMath.zeroToTwoPi(viewModel.rotateInitialCameraAngle - angleDifference);
    var camera = viewModel.terria.scene.camera;
    var oldTransform;

    if (defined$5(viewModel.rotateFrame)) {
      oldTransform = Matrix4.clone(camera.transform, oldTransformScratch);
      camera.lookAtTransform(viewModel.rotateFrame);
    }

    var currentCameraAngle = -camera.heading;
    camera.rotateRight(newCameraAngle - currentCameraAngle);

    if (defined$5(viewModel.rotateFrame)) {
      camera.lookAtTransform(oldTransform);
    } // viewModel.terria.cesium.notifyRepaintRequired();

  };

  viewModel.rotateMouseUpFunction = function (e) {
    viewModel.isRotating = false;
    document.removeEventListener('mousemove', viewModel.rotateMouseMoveFunction, false);
    document.removeEventListener('mouseup', viewModel.rotateMouseUpFunction, false);
    viewModel.rotateMouseMoveFunction = undefined;
    viewModel.rotateMouseUpFunction = undefined;
  };

  document.addEventListener('mousemove', viewModel.rotateMouseMoveFunction, false);
  document.addEventListener('mouseup', viewModel.rotateMouseUpFunction, false);
}

/* eslint-disable no-unused-vars */
var Cesium$a = window.Cesium; // 用于Cesium.js src引入

var defined$6 = Cesium$a.defined,
    Event = Cesium$a.Event,
    knockout$6 = Cesium$a.knockout,
    DeveloperError$2 = Cesium$a.DeveloperError;
var CesiumEvent = Event;
/**
 * @alias CesiumNavigation
 * @constructor
 * @example
 * new CesiumNavigation(viewer, {
        enableCompass: true, // 罗盘
        enableZoomControls: true, // 缩放控制器
        enableDistanceLegend: true, // 比例尺
        enableCompassOuterRing: true, // 罗盘外环
        view: { // 初始化视角
          "x": 109.7884118470029,
          "y": 39.590384952017764,
          "z": 1565.7899788867958,
          "heading": 331.1978494043747,
          "pitch": -8.45296669256617,
          "roll": 0.00043210090111595544
        }
    });

 * @param {Cesium.Viewer} viewerCesiumWidget 地图viewer对象 
 * @param {Object} options 相关配置
 * @param {Boolean} [options.enableCompass=true] 是否创建罗盘
 * @param {Object} [options.compass] 罗盘样式设置
 * @param {Object} [options.compass.style=='leftBottom'] 罗盘位置
 * @param {Boolean} [options.enableZoomControls=true] 是否创建缩放控制器
 * @param {Boolean} [options.enableDistanceLegend=true] 是否创建比例尺
 * @param {Object} [options.distanceLegend] 比例尺样式设置
 * @param {Object} [options.distanceLegend.style=='leftBottom'] 比例尺位置
 * @param {Boolean} [options.enableCompassOuterRing=true] 是否创建罗盘外环
 * @param {Object} [options.view] 初始化视角
 */

var CesiumNavigation = function CesiumNavigation(viewerCesiumWidget, options) {
  initialize.apply(this, arguments);
  this._onDestroyListeners = [];
};

CesiumNavigation.prototype.distanceLegendViewModel = undefined;
CesiumNavigation.prototype.navigationViewModel = undefined;
CesiumNavigation.prototype.navigationDiv = undefined;
CesiumNavigation.prototype.distanceLegendDiv = undefined;
CesiumNavigation.prototype.terria = undefined;
CesiumNavigation.prototype.container = undefined;
CesiumNavigation.prototype._onDestroyListeners = undefined;
CesiumNavigation.prototype._navigationLocked = false;

CesiumNavigation.prototype.setNavigationLocked = function (locked) {
  this._navigationLocked = locked;
  this.navigationViewModel.setNavigationLocked(this._navigationLocked);
};

CesiumNavigation.prototype.getNavigationLocked = function () {
  return this._navigationLocked;
}; // CesiumNavigation.prototype.bindObserver = function () {
//   let MutationObserver = window.MutationObserver || window.WebKitMutationObserver || window.MozMutationObserver;
//   let dom_legend = document.querySelector('.distance-legend');
//   let dom_compass = document.querySelector('.compass');
//   let dom_controls = document.querySelector('.navigation-controls');
//   let observer = new MutationObserver((mutationList) => {
//     let width_legend = getComputedStyle(dom_legend).getPropertyValue('width');
//     let height_legend = getComputedStyle(dom_legend).getPropertyValue('height');
//     let width_compass = getComputedStyle(dom_compass).getPropertyValue('width');
//     let height_compass = getComputedStyle(dom_compass).getPropertyValue('height');
//     let width_controls = getComputedStyle(dom_controls).getPropertyValue('width');
//     let height_controls = getComputedStyle(dom_controls).getPropertyValue('height')
//     width_legend = parseInt(width_legend);
//     if (width_legend <= 180) {
//       dom_legend.style.display = "none";
//     } else {
//       dom_legend.style.display = "flex";
//     }
//     width_compass = parseInt(width_compass);
//     height_compass = parseInt(height_compass);
//     if (width_compass <= 180 || height_compass <=180) {
//       dom_compass.style.display = "none";
//     } else {
//       dom_legend.style.display = "flex";
//     }
//     width_controls = parseInt(width_controls);
//     height_controls = parseInt(height_controls);
//     if (width_controls <= 180) {
//       dom_legend.style.display = "none";
//     } else {
//       dom_legend.style.display = "flex";
//     }
//   })
//   observer.observe(element, { attributes: true, attributeFilter: ['style'], attributeOldValue: true })
// }

/**
 * 是否显示
 * @param {Boolean} visible 
 */


CesiumNavigation.prototype.setVisible = function (visible) {
  var dom_legend = document.querySelector('.distance-legend');
  var dom_compass = document.querySelector('.compass');
  var dom_controls = document.querySelector('.navigation-controls');
  dom_legend.style.display = visible ? "block" : "none";
  dom_compass.style.display = visible ? "block" : "none";
  dom_controls.style.display = visible ? "block" : "none";
};
/**
 * 销毁
 */


CesiumNavigation.prototype.destroy = function () {
  if (defined$6(this.navigationViewModel)) {
    this.navigationViewModel.destroy();
  }

  if (defined$6(this.distanceLegendViewModel)) {
    this.distanceLegendViewModel.destroy();
  }

  if (defined$6(this.navigationDiv)) {
    this.navigationDiv.parentNode.removeChild(this.navigationDiv);
  }

  delete this.navigationDiv;

  if (defined$6(this.distanceLegendDiv)) {
    this.distanceLegendDiv.parentNode.removeChild(this.distanceLegendDiv);
  }

  delete this.distanceLegendDiv;

  if (defined$6(this.container)) {
    this.container.parentNode.removeChild(this.container);
  }

  delete this.container;

  for (var i = 0; i < this._onDestroyListeners.length; i++) {
    this._onDestroyListeners[i]();
  }
};

CesiumNavigation.prototype.addOnDestroyListener = function (callback) {
  if (typeof callback === 'function') {
    this._onDestroyListeners.push(callback);
  }
};
/**
 
 */


function initialize(viewerCesiumWidget, options) {
  if (!defined$6(viewerCesiumWidget)) {
    throw new DeveloperError$2('CesiumWidget or Viewer is required.');
  }

  var cesiumWidget = defined$6(viewerCesiumWidget.cesiumWidget) ? viewerCesiumWidget.cesiumWidget : viewerCesiumWidget; // 构件导航球容器

  var container = document.createElement('div');
  container.className = 'cesium-widget-cesiumNavigationContainer';
  container.style.position = 'absolute';
  container.style.bottom = '0px';
  container.style.zIndex = '99';
  var mapDom = document.getElementById(viewerCesiumWidget._container.id);
  mapDom.appendChild(container);
  this.container = container;
  this.terria = viewerCesiumWidget;
  this.terria.options = defined$6(options) ? options : {}; // 定义viewer的事件 供其它模块调用

  this.terria.afterWidgetChanged = new CesiumEvent();
  this.terria.beforeWidgetChanged = new CesiumEvent(); // 比例尺

  this.distanceLegendDiv = document.createElement('div');
  container.appendChild(this.distanceLegendDiv);
  this.distanceLegendDiv.setAttribute('id', 'distanceLegendDiv');
  var distanceStyleAttr = options.distanceLegend && options.distanceLegend.style || "leftBottom";
  distanceStyleAttr = typeof distanceStyleAttr == "string" ? getDistanceStyleByAttr(distanceStyleAttr) : distanceStyleAttr;
  this.distanceLegendViewModel = DistanceLegendViewModel.create({
    container: this.distanceLegendDiv,
    style: distanceStyleAttr,
    terria: this.terria,
    enableDistanceLegend: this.terria.options.enableDistanceLegend == undefined ? true : this.terria.options.enableDistanceLegend
  }); // 指北针及缩放按钮

  this.navigationDiv = document.createElement('div');
  this.navigationDiv.setAttribute('id', 'navigationDiv');
  container.appendChild(this.navigationDiv);
  var compassStyleAttr = options.compass && options.compass.style || "leftBottom";
  compassStyleAttr = typeof compassStyleAttr == "string" ? getCompassStyleByAttr(compassStyleAttr) : compassStyleAttr;
  this.navigationViewModel = NavigationViewModel.create({
    container: this.navigationDiv,
    terria: this.terria,
    style: compassStyleAttr,
    enableZoomControls: this.terria.options.enableZoomControls == undefined ? true : this.terria.options.enableZoomControls,
    enableCompass: this.terria.options.enableCompass == undefined ? true : this.terria.options.enableCompass
  });
  registerKnockoutBindings();
}

function getDistanceStyleByAttr(type) {
  type = type || "leftBottom";
  var defaultStyle = {};

  if (type == "leftBottom") {
    defaultStyle = {
      left: "0px",
      bottom: "4px"
    };
  } else if (type == "leftTop") {
    defaultStyle = {
      left: "0px",
      top: "20px"
    };
  } else if (type == "rightBottom") {
    defaultStyle = {
      right: "20px",
      bottom: "4px"
    };
  } else if (type == "rightTop") {
    defaultStyle = {
      right: "20px",
      top: "20px"
    };
  } else {
    defaultStyle = type;
  }

  defaultStyle.zIndex = 99999;
  return defaultStyle;
}

function getCompassStyleByAttr(type) {
  type = type || "rightTop";
  var defaultStyle = {};

  if (type == "leftBottom") {
    defaultStyle = {
      left: "20px",
      bottom: "60px"
    };
  } else if (type == "leftTop") {
    defaultStyle = {
      left: "20px",
      top: "20px"
    };
  } else if (type == "rightBottom") {
    defaultStyle = {
      right: "20px",
      bottom: "60px"
    };
  } else if (type == "rightTop") {
    defaultStyle = {
      right: "20px",
      top: "20px"
    };
  } else {
    defaultStyle = type;
  }

  defaultStyle.zIndex = 99999;
  return defaultStyle;
}

/**
 * 缩放工具
 * @class
 * 
 */
var ZoomTool = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象
   * @param {Object} opt 基础配置
   * @param {Object} opt.step 缩放步长
   */
  function ZoomTool(viewer, opt) {
    _classCallCheck(this, ZoomTool);

    this.viewer = viewer;
    this.opt = opt || {};
    /**
     * @property {Number} step 缩放步长
     */

    this.step = this.opt.step || 0.5;
    this.forwardAmount = null;
    this.backwardAmount = null;
    /**
     * @property {Cesium.Cartesian3} position 相机位置
     */

    this.position = null;
  }
  /**
   * 向前移动
   */


  _createClass(ZoomTool, [{
    key: "forward",
    value: function forward() {
      var amount;

      if (this.backwardAmount) {
        amount = this.backwardAmount;
        this.backwardAmount = null;
      } else {
        amount = this.computeLength() || 0;
        amount = amount * this.step;
      }

      this.viewer.camera.moveForward(amount);
      this.forwardAmount = amount;
    }
    /**
     * 向后移动
     */

  }, {
    key: "backward",
    value: function backward() {
      var amount;

      if (this.forwardAmount) {
        amount = this.forwardAmount;
        this.forwardAmount = null;
      } else {
        amount = this.computeLength() || 0;
        amount = amount * this.step;
      }

      this.viewer.camera.moveBackward(amount);
      this.backwardAmount = amount;
    } // 计算相机距离

  }, {
    key: "computeLength",
    value: function computeLength() {
      this.position = this.viewer.camera.position;
      var dir = this.viewer.camera.direction;
      var lnglat = Cesium.Cartographic.fromCartesian(this.position);
      var height = lnglat.height; // 求出相机和地形的高度差

      var ray = new Cesium.Ray(this.position.clone(), dir.clone());
      var intersec = this.viewer.scene.globe.pick(ray, this.viewer.scene);
      if (!intersec) return undefined;
      var cctgc = Cesium.Cartographic.fromCartesian(intersec);
      height = lnglat.height - cctgc.height;
      dir = Cesium.Cartesian3.normalize(dir, new Cesium.Cartesian3());
      var reverseZ = new Cesium.Cartesian3(0, 0, -1);
      var cosAngle = Cesium.Cartesian3.dot(dir, reverseZ);
      var angle = Math.asin(cosAngle);
      var length = height / Math.cos(angle);
      return length;
    }
  }]);

  return ZoomTool;
}();

var RightTool = /*#__PURE__*/function () {
  function RightTool(viewer, opt) {
    _classCallCheck(this, RightTool);

    opt = opt || {};
    var defaultVal = {
      lnglat: true,
      cameraView: true,
      depth: true
    };
    this.opt = Object.assign(defaultVal, opt);

    if (!viewer) {
      console.log("缺少viewer对象！");
      return;
    }

    this.viewer = viewer;
    var id = this.viewer.container.id;
    this.mapContainer = document.getElementById(id);
    this.rightClickHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this.randomId = new Date().getTime() + "" + Math.ceil(Math.random() * 10000);
    var tooEle = this.createElement('div', {
      "class": 'vis3d-right-tool',
      id: "vis3d-right-tool-".concat(this.randomId),
      html: "<ul></ul>"
    });
    this.mapContainer.appendChild(tooEle);
    var toolContainer = document.getElementById("vis3d-right-tool-".concat(this.randomId));
    this.toolMenu = toolContainer.querySelector("ul"); // 点击其它地方 关闭面板

    document.addEventListener("click", function () {
      var tool = document.getElementsByClassName('vis3d-right-tool');
      if (tool[0]) tool[0].style.display = "none";
    });
    document.getElementById("vis3d-right-tool-".concat(this.randomId)).addEventListener("click", function (event) {
      event.stopPropagation(); //  阻止事件冒泡
    });

    if (this.opt.lnglat) {
      this.crateLnglatTool();
    }

    if (this.opt.cameraView) {
      this.createCameraViewTool();
    }

    if (this.opt.depth) {
      this.crateDepthTool();
    }

    this.bindHandler();
    this._clickPX = null;
    this.scale = this.opt.scale || [1, 1];
  }

  _createClass(RightTool, [{
    key: "createElement",
    value: function createElement(eleType, opt) {
      opt = opt || {};
      var ele = document.createElement(eleType);
      if (opt["class"]) ele.className = opt["class"];
      if (opt.id) ele.setAttribute("id", opt.id);
      ele.innerHTML = opt.html;
      return ele;
    }
  }, {
    key: "crateLnglatTool",
    value: function crateLnglatTool() {
      var that = this;
      var dom = this.createElement('li', {
        id: "right-tool-lnglat-".concat(this.randomId),
        "class": "right-tool-lnglat",
        html: "<span style=\"font-weight:bold;\">\u5F53\u524D\u5750\u6807</span>"
      });
      this.toolMenu.appendChild(dom);
      var rightToolLnglat = document.getElementById("right-tool-lnglat-".concat(this.randomId));
      rightToolLnglat.addEventListener('click', function (evt) {
        if (!that._clickPX) return;
        var picks = that.viewer.scene.drillPick(that._clickPX);
        that.viewer.scene.render();
        var cartesian;
        var isOn3dtiles = false;

        for (var i = 0; i < picks.length; i++) {
          if (picks[i] && picks[i].primitive && picks[i].primitive instanceof Cesium.Cesium3DTileset) {
            //模型上拾取
            isOn3dtiles = true;
            break;
          }
        }

        if (isOn3dtiles) {
          cartesian = that.viewer.scene.pickPosition(that._clickPX);
        } else {
          var ray = that.viewer.camera.getPickRay(that._clickPX);
          if (!ray) return null;
          cartesian = that.viewer.scene.globe.pick(ray, that.viewer.scene);
        }

        var ctgc = Cesium.Cartographic.fromCartesian(cartesian.clone());
        var lng = Cesium.Math.toDegrees(ctgc.longitude);
        var lat = Cesium.Math.toDegrees(ctgc.latitude);
        var height = ctgc.height;
        var title = "该点坐标";
        var resultC3 = "[".concat(Number(cartesian.x), " , ").concat(Number(cartesian.y), " , ").concat(Number(cartesian.z), "]");
        var resultJWD = "[".concat(Number(lng).toFixed(6), " , ").concat(Number(lat).toFixed(6), " , ").concat(Number(height).toFixed(2), "]");
        var result = "\n                \u4E16\u754C\u5750\u6807\uFF1A\n                <div>".concat(resultC3, "</div>\n                \u7ECF\u7EAC\u5EA6\uFF1A\n                <div>").concat(resultJWD, "</div>\n            ");
        that.crerateResultHtml(title, result);
      });
    }
  }, {
    key: "createCameraViewTool",
    value: function createCameraViewTool() {
      var that = this;
      var dom = this.createElement('li', {
        "class": "right-tool-view",
        id: "right-tool-view-".concat(this.randomId),
        html: "<span>\u76F8\u673A\u89C6\u89D2</span>"
      });
      this.toolMenu.appendChild(dom);
      var rightToolView = document.getElementById("right-tool-view-".concat(this.randomId));
      rightToolView.addEventListener('click', function () {
        var camera = that.viewer.camera;
        var position = camera.position;
        var heading = camera.heading;
        var pitch = camera.pitch;
        var roll = camera.roll;
        var lnglat = Cesium.Cartographic.fromCartesian(position);
        var str = "\n                <div>{</div>\n                <ul style=\"margin-left:10px;\">\n                    <li>\n                        <span>\n                            \"x\" : ".concat(Cesium.Math.toDegrees(lnglat.longitude), ",\n                        </span>\n                    </li>\n                    <li>\n                        <span>\n                            \"y\" : ").concat(Cesium.Math.toDegrees(lnglat.latitude), ",\n                        </span>\n                    </li>\n                    <li>\n                        <span>\n                            \"z\" : ").concat(lnglat.height, ",\n                        </span>\n                    </li>\n                    <li>\n                        <span>\n                            \"heading\" : ").concat(Cesium.Math.toDegrees(heading), ",\n                        </span>\n                    </li>\n                    <li>\n                        <span>\n                            \"pitch\" : ").concat(Cesium.Math.toDegrees(pitch), ",\n                        </span>\n                    </li>\n                    <li>\n                        <span>\n                        \"roll\" : ").concat(Cesium.Math.toDegrees(roll), "\n                        </span>\n                    </li>\n                </ul>\n                <div>}</div>\n            ");
        var title = "当前相机视角";
        that.crerateResultHtml(title, str);
      });
    }
  }, {
    key: "crateDepthTool",
    value: function crateDepthTool() {
      var that = this;
      var oldDepth = this.viewer.scene.globe.depthTestAgainstTerrain;
      var depthVal = !oldDepth ? "深度检测（开）" : "深度检测（关）";
      var dom = this.createElement('li', {
        "class": "right-tool-view",
        id: "right-tool-view-".concat(this.randomId),
        html: " <span class=\"right-tool-depth\" id=\"right-tool-depth-".concat(this.randomId, "\">\n                        ").concat(depthVal, "\n                    </span>")
      });
      this.toolMenu.appendChild(dom);
      var depthDom = document.getElementById("right-tool-depth-".concat(this.randomId));
      depthDom.addEventListener('click', function (evt, res) {
        var tool = document.getElementsByClassName('vis3d-right-tool');
        if (tool[0]) tool[0].style.display = "none";
        var text = depthDom.innerText;

        if (text.indexOf("开") != -1) {
          // 表示当前是开启状态
          depthDom.innerText = "深度检测（关）";
          that.viewer.scene.globe.depthTestAgainstTerrain = true;
        } else {
          depthDom.innerText = "深度检测（开）";
          that.viewer.scene.globe.depthTestAgainstTerrain = false;
        }
      });
    }
  }, {
    key: "refreshDepthVal",
    value: function refreshDepthVal() {
      var oldDepth = this.viewer.scene.globe.depthTestAgainstTerrain;
      var depthVal = !oldDepth ? "深度检测（开）" : "深度检测（关）";
      document.getElementById("right-tool-depth-".concat(this.randomId)).innerHTML = depthVal;
    }
  }, {
    key: "bindHandler",
    value: function bindHandler() {
      var that = this;
      this.rightClickHandler.setInputAction(function (evt) {
        evt.position.x = evt.position.x / that.scale[0];
        evt.position.y = evt.position.y / that.scale[1];
        var pick = that.viewer.scene.pick(evt.position);
        var ent;

        if (pick && pick.primitive && !(pick.primitive instanceof Cesium.Cesium3DTileset)) {
          // 拾取图元
          ent = pick.primitive;
        }

        if (pick && pick.id && pick.id instanceof Cesium.Entity) {
          ent = pick.id;
        }

        if (ent && ent.objId) return; // 此处要防止和plot的右键菜单冲突

        that.refreshDepthVal();
        that._clickPX = evt.position; // 获取其相对于屏幕左上角的位置

        var bcr = that.mapContainer.getBoundingClientRect();
        var dom = document.getElementById("vis3d-right-tool-".concat(that.randomId));
        dom.style.left = Number(evt.position.x + 10) + bcr.left + "px";
        dom.style.top = Number(evt.position.y + 10) + bcr.top + "px";
        dom.style.display = "block";
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
    }
  }, {
    key: "destroy",
    value: function destroy() {
      if (this.rightClickHandler) {
        this.rightClickHandler.destroy();
        this.rightClickHandler = null;
      }

      var dom = document.getElementById("vis3d-right-tool-".concat(this.randomId));
      dom.parentNode.removeChild(dom);
      var dom2 = document.getElementById("vis3d-right-content-".concat(this.randomId));
      dom2.parentNode.removeChild(dom2);
    } // 构建结果面板

  }, {
    key: "crerateResultHtml",
    value: function crerateResultHtml(title, result) {
      // 关闭菜单栏
      var tool = document.getElementsByClassName('vis3d-right-tool');
      if (tool[0]) tool[0].style.display = "none";
      var resele = this.createElement('div', {
        "class": "vis3d-right-content",
        id: "vis3d-right-content-".concat(this.randomId),
        html: "\n            <span class=\"right-content-close\" id=\"right-content-close-".concat(this.randomId, "\" alt=\"\" title=\"\u70B9\u51FB\u5173\u95ED\">x</span>\n            <div class=\"right-content-result scrollbar\">\n                <div class=\"content-result-title\" style=\"font-weight:bold;\">").concat(title, "\uFF1A</div>\n                <div class=\"content-result-line\"></div>\n                <div class=\"content-result-info\">").concat(result, "</div>\n            </div>\n            ")
      });
      var body = document.getElementsByTagName('body')[0];
      body.appendChild(resele);
      var rightContentClose = document.getElementById("right-content-close-".concat(this.randomId));
      var rightContent = document.getElementById("vis3d-right-content-".concat(this.randomId));
      rightContentClose.addEventListener('click', function () {
        if (rightContent) rightContent.parentNode.removeChild(rightContent);
      });
    } // 扩展右键菜单

  }, {
    key: "extend",
    value: function extend(opt) {
      var id = opt.id || new Date().getTime();
      var customId = opt.id || "right-tool-extend-".concat(id);
      var dom = this.createElement('li', {
        id: customId,
        "class": opt["class"] || 'right-tool-extend',
        html: opt.content
      });
      this.toolMenu.appendChild(dom);
      document.getElementById(customId).addEventListener('click', function () {
        var tool = document.getElementsByClassName('vis3d-right-tool');
        if (tool[0]) tool[0].style.display = "none";
        if (opt.click) opt.click();
      });
    }
  }]);

  return RightTool;
}();

/**
 * 底部鼠标及相机信息提示栏
 * @class
 */

var LnglatTool = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.viewer} viewer 地图viewer对象 
   * @param {Object} [opt] 其它配置 
   */
  function LnglatTool(viewer, opt) {
    _classCallCheck(this, LnglatTool);

    this.viewer = viewer;
    this.opt = opt || {};
    this.moveHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this.initHtml();
    this.bindMouseMoveHandler();
    this.ellipsoid = this.viewer.scene.globe.ellipsoid;
    this.scale = this.opt.scale || [1, 1];
  }

  _createClass(LnglatTool, [{
    key: "bindMouseMoveHandler",
    value: function bindMouseMoveHandler() {
      var that = this;
      this.moveHandler.setInputAction(function (evt) {
        //单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.endPosition);
        if (!cartesian) return;
        var lnglat = that.ellipsoid.cartesianToCartographic(cartesian);
        var lat = Cesium.Math.toDegrees(lnglat.latitude);
        var lng = Cesium.Math.toDegrees(lnglat.longitude);
        var height = lnglat.height;
        var cameraV = that.getCameraView();
        that.setHtml({
          lng: lng,
          lat: lat,
          height: height
        }, cameraV);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.moveHandler) {
        this.moveHandler.destroy();
        this.moveHandler = null;
      }

      var doms = document.getElementsByClassName("vis3d-lnglatNavigation");
      doms[0].parentNode.removeChild(doms[0]);
    }
    /**
     * 控制坐标显示隐藏
     * @param {Boolean} visible true显示/false隐藏
     */

  }, {
    key: "setVisible",
    value: function setVisible(visible) {
      var doms = document.getElementsByClassName("vis3d-lnglatNavigation");
      if (!doms[0]) return;
      var dom = doms[0];
      /* const display = dom.style.display;
      const displayVisible = display == "block" ? true : false; */

      dom.style.display = visible ? "block" : "none";
    }
  }, {
    key: "initHtml",
    value: function initHtml() {
      var id = this.viewer.container.id;
      var mapDom = document.getElementById(id);
      var ele = document.createElement("div");
      ele.className = 'vis3d-lnglatNavigation';
      ele.innerHTML = " <ul>\n                            <li style=\"width:120px;\"></li>   \n                            <li style=\"width:120px;\"></li>\n                            <li style=\"width:120px;\"></li>\n                            <li></li>\n                            <li></li>\n                            <li></li>\n                            <li></li>\n                        <ul>";
      mapDom.appendChild(ele);
    }
  }, {
    key: "getCatesian3FromPX",
    value: function getCatesian3FromPX(px) {
      if (!px) return;
      px.x = px.x / this.scale[0];
      px.y = px.y / this.scale[1];
      var pick = this.viewer.scene.pick(px);
      var cartesian = undefined;

      if (pick && pick.primitive instanceof Cesium.Cesium3DTileset) {
        cartesian = this.viewer.scene.pickPosition(px);
      } else {
        var ray = this.viewer.camera.getPickRay(px);
        if (!ray) return null;
        cartesian = this.viewer.scene.globe.pick(ray, this.viewer.scene);
        //sin add 20240718
        if (!cartesian) {
          cartesian = this.viewer.camera.pickEllipsoid(
              px,
              this.viewer.scene.globe.ellipsoid
          );
        }
      }

      return cartesian;
    }
  }, {
    key: "setHtml",
    value: function setHtml(latlngOpt, cameraView) {
      var lng = Number(latlngOpt.lng).toFixed(6);
      var lat = Number(latlngOpt.lat).toFixed(6);
      var height = Number(latlngOpt.height).toFixed(2);
      var heading = Number(cameraView.heading).toFixed(2);
      var pitch = Number(cameraView.pitch).toFixed(2);
      var roll = Number(cameraView.roll).toFixed(2);
      var z = Number(cameraView.z).toFixed(2);
      var eles = document.getElementsByClassName('vis3d-lnglatNavigation');
      if (!eles || eles.length < 1) return;
      var ele = eles[0];
      var lis = ele.children[0].children;
      lis[0].innerHTML = "\u7ECF\u5EA6\uFF1A".concat(lng);
      lis[1].innerHTML = "\u7EAC\u5EA6\uFF1A".concat(lat);
      lis[2].innerHTML = "\u9AD8\u5EA6\uFF1A".concat(height);
      lis[3].innerHTML = "\u504F\u8F6C\u89D2\uFF1A".concat(heading);
      lis[4].innerHTML = "\u4EF0\u4FEF\u89D2\uFF1A".concat(pitch);
      lis[5].innerHTML = "\u7FFB\u6EDA\u89D2\uFF1A".concat(roll);
      lis[6].innerHTML = "\u76F8\u673A\u9AD8\u5EA6\uFF1A".concat(z);
    }
  }, {
    key: "getCameraView",
    value: function getCameraView() {
      var camera = this.viewer.camera;
      var position = camera.position;
      var heading = camera.heading;
      var pitch = camera.pitch;
      var roll = camera.roll;
      var lnglat = Cesium.Cartographic.fromCartesian(position);
      return {
        "x": Cesium.Math.toDegrees(lnglat.longitude),
        "y": Cesium.Math.toDegrees(lnglat.latitude),
        "z": lnglat.height,
        "heading": Cesium.Math.toDegrees(heading),
        "pitch": Cesium.Math.toDegrees(pitch),
        "roll": Cesium.Math.toDegrees(roll)
      };
    }
  }]);

  return LnglatTool;
}();

/**
 * 卷帘对比
 * @description 卷帘对比类
 * @class 
 */
var LayerSplit = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt
   * @param {Cesium.Layer} opt.layer 需要对比的图层
   *  
   */
  function LayerSplit(viewer, opt) {
    _classCallCheck(this, LayerSplit);

    this.viewer = viewer;
    this.slider = null;
    this.handler = null;
    this.moveActive = false;
    this.opt = opt || {};
    /**
    * @property {Cesium.Layer} layer 对比的图层
    */

    this.layer = this.opt.layer;
    /*   if (!this.layer) return; */

    this.splitDirection = this.opt.splitDirection == undefined ? 1 : this.opt.splitDirection;
    if (this.layer) this.layer.splitDirection = this.splitDirection; // 默认右侧分割

    this.mapContainer = this.viewer.container;
    this.init();
  }

  _createClass(LayerSplit, [{
    key: "init",
    value: function init() {
      var that = this;
      this.slider = window.document.createElement("div");
      this.slider.setAttribute("id", "layer-split");
      this.slider.style.height = "100%";
      this.slider.style.width = "5px";
      this.slider.style.position = "absolute";
      this.slider.style.left = "50%";
      /*  this.slider.style.zIndex = "999"; */

      this.slider.style.top = "0px";
      this.slider.style.background = "rgba(255,255,255,0.5)";
      this.mapContainer.appendChild(this.slider);
      this.handler = new window.Cesium.ScreenSpaceEventHandler(this.slider);
      this.viewer.scene.splitPosition = 0.5;
      this.handler.setInputAction(function () {
        that.moveActive = true;
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.handler.setInputAction(function () {
        that.moveActive = true;
      }, Cesium.ScreenSpaceEventType.PINCH_START);
      this.handler.setInputAction(function (movement) {
        that.move(movement);
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
      this.handler.setInputAction(function (movement) {
        that.move(movement);
      }, Cesium.ScreenSpaceEventType.PINCH_MOVE);
      this.handler.setInputAction(function () {
        that.moveActive = false;
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
      this.handler.setInputAction(function () {
        that.moveActive = false;
      }, Cesium.ScreenSpaceEventType.PINCH_END);
    }
  }, {
    key: "move",
    value: function move(movement) {
      if (!this.moveActive) {
        return;
      }

      var relativeOffset = movement.endPosition.x;
      var splitPosition = (this.slider.offsetLeft + relativeOffset) / this.slider.parentElement.offsetWidth;
      this.slider.style.left = 100.0 * splitPosition + '%';
      this.viewer.scene.splitPosition = splitPosition;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.slider) {
        this.slider.parentNode.removeChild(this.slider);
        this.slider = null;
      }

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      this.layer.splitDirection = window.Cesium.SplitDirection.NONE;
    }
    /**
     * 设置对比图层
     * @param {Cesium.Layer} layer 图层对象 
     */

  }, {
    key: "setLayer",
    value: function setLayer(layer) {
      if (!layer) return;
      this.layer.splitDirection = window.Cesium.SplitDirection.NONE;
      this.layer = layer;
      this.layer.splitDirection = this.splitDirection;
    }
  }]);

  return LayerSplit;
}();

/**
 * 聚合
 * @description 图标、文字等entity聚合类，通过此对象添加的entity都会自动聚合
 * @class
 */
var Cluster = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt
   * @param {Boolean} opt.enabled 是否聚合
   * @param {Number}  opt.pixelRange 聚合的像素范围
   * @param {Number}  opt.minimumClusterSize 聚合的对象最小数量
   * @param {Array}  opt.conditions 聚合的条件数组
   */
  function Cluster(viewer, opt) {
    _classCallCheck(this, Cluster);

    this.viewer = viewer;
    this.opt = opt || {};
    var defaultOpt = {
      enabled: true,
      pixelRange: 60,
      minimumClusterSize: 3
    };
    var defaultConditions = [{
      number: 10,
      // < 10 为绿色
      color: 'rgba(0,255,0,0.3)'
    }, {
      number: 50,
      // 10 - 50 为黄色色
      color: 'rgba(255,255,0,0.3)'
    }, {
      number: Number.MAX_VALUE,
      // >50 红色
      color: 'rgba(255,0,0,0.3)'
    }];
    this.conditions = this.opt.conditions || defaultConditions;
    this.conditions = this.conditions.sort(function (a, b) {
      return a.number - b.number;
    });
    this.opt = Object.assign(defaultOpt, this.opt);
    /**
     * @property {Cesium.CustomDataSource} clusterDataSource 实体容器
     */

    this.clusterDataSource = new Cesium.CustomDataSource("clusterDataSource");
    this.clusterDataSource.clustering.enabled = this.opt.enabled;
    this.clusterDataSource.clustering.pixelRange = this.opt.pixelRange;
    this.clusterDataSource.clustering.minimumClusterSize = this.opt.minimumClusterSize;
    this.viewer.dataSources.add(this.clusterDataSource);
    this.bindCluster();
  }
  /**
   * 添加实体
   * @param {Cesium.Entity} ent 实体对象
   */


  _createClass(Cluster, [{
    key: "add",
    value: function add(ent) {
      if (ent) this.clusterDataSource.entities.add(ent);
    }
    /**
    * 移除实体
    * @param {Cesium.Entity} ent 实体对象
    */

  }, {
    key: "remove",
    value: function remove(ent) {
      if (ent) this.clusterDataSource.entities.remove(ent);
    }
    /**
     * 清空
     */

  }, {
    key: "clear",
    value: function clear() {
      this.clusterDataSource.entities.removeAll();
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      this.clusterDataSource.entities.removeAll();
      this.viewer.dataSources.remove(this.clusterDataSource);
    }
    /**
     * 开启聚合
     */

  }, {
    key: "open",
    value: function open() {
      this.clusterDataSource.enableCluster = true;
    }
    /**
     * 关闭聚合
     */

  }, {
    key: "close",
    value: function close() {
      this.clusterDataSource.enableCluster = false;
    }
  }, {
    key: "bindCluster",
    value: function bindCluster() {
      var that = this;
      this.clusterDataSource.clustering.clusterEvent.addEventListener(function (clusteredEntities, cluster) {
        var length = clusteredEntities.length;
        cluster.label.show = false;
        cluster.billboard.show = true;
        cluster.billboard.id = cluster.label.id;
        cluster.billboard.verticalOrigin = Cesium.VerticalOrigin.BOTTOM;
        cluster.billboard.image = that.getClusterImg(length);
      }, window);
    }
  }, {
    key: "getClusterImg",
    value: function getClusterImg(length, fun) {
      var canvas = document.createElement("canvas"); // 设置canvas的大小

      canvas.width = 64;
      canvas.height = 64; // 获得canva上下文环境

      var ctx = canvas.getContext("2d");
      ctx.beginPath();
      ctx.arc(32, 32, 20, 0, Math.PI * 2, true);
      ctx.closePath();
      var color = this.getColorByLength(length) || "rgba(0,255,0,0.3)";
      ctx.fillStyle = color;
      ctx.fill();
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.font = "16px Arial";
      ctx.fillStyle = "#ffffff";
      var text = length + "";
      ctx.fillText(text, canvas.width / 2, canvas.height / 2 + 2);
      var image = canvas.toDataURL("image/png");
      return image;
    }
  }, {
    key: "getColorByLength",
    value: function getColorByLength(number) {
      var res = this.conditions.find(function (cd) {
        return cd.number > number;
      });
      return res.color;
    }
  }]);

  return Cluster;
}();

var SkyboxGround = /*#__PURE__*/function () {
  function SkyboxGround(options) {
    _classCallCheck(this, SkyboxGround);

    /**
         * 近景天空盒
         * @type Object
         * @default undefined
         */
    this.sources = options.sources;
    this._sources = undefined;
    /**
     * Determines if the sky box will be shown.
     *
     * @type {Boolean}
     * @default true
     */

    this.show = Cesium.defaultValue(options.show, true);
    this._command = new Cesium.DrawCommand({
      modelMatrix: Cesium.Matrix4.clone(Cesium.Matrix4.IDENTITY),
      owner: this
    });
    this._cubeMap = undefined;
    this._attributeLocations = undefined;
    this._useHdr = undefined; //片元着色器，直接从源码复制

    this.SkyBoxFS = "\n            uniform samplerCube u_cubeMap;\n            in vec3 v_texCoord;\n            void main()\n            {\n                vec4 color = textureCube(u_cubeMap, normalize(v_texCoord));\n                out_FragColor = vec4(czm_gammaCorrect(color).rgb, czm_morphTime);\n            }\n            "; //顶点着色器有修改，主要是乘了一个旋转矩阵

    this.SkyBoxVS = "\n            in vec3 position;\n            out vec3 v_texCoord;\n            uniform mat3 u_rotateMatrix;\n            void main()\n            {\n                vec3 p = czm_viewRotation * u_rotateMatrix * (czm_temeToPseudoFixed * (czm_entireFrustum.y * position));\n                gl_Position = czm_projection * vec4(p, 1.0);\n                v_texCoord = position.xyz;\n            }\n            ";
  }

  _createClass(SkyboxGround, [{
    key: "update",
    value: function update(frameState, useHdr) {
      var that = this;

      if (!this.show) {
        return undefined;
      }

      if (frameState.mode !== Cesium.SceneMode.SCENE3D && frameState.mode !== Cesium.SceneMode.MORPHING) {
        return undefined;
      }

      if (!frameState.passes.render) {
        return undefined;
      }

      var context = frameState.context;

      if (this._sources !== this.sources) {
        this._sources = this.sources;
        var sources = this.sources;

        if (!Cesium.defined(sources.positiveX) || !Cesium.defined(sources.negativeX) || !Cesium.defined(sources.positiveY) || !Cesium.defined(sources.negativeY) || !Cesium.defined(sources.positiveZ) || !Cesium.defined(sources.negativeZ)) {
          throw new Cesium.DeveloperError("this.sources is required and must have positiveX, negativeX, positiveY, negativeY, positiveZ, and negativeZ properties.");
        }

        if (_typeof(sources.positiveX) !== _typeof(sources.negativeX) || _typeof(sources.positiveX) !== _typeof(sources.positiveY) || _typeof(sources.positiveX) !== _typeof(sources.negativeY) || _typeof(sources.positiveX) !== _typeof(sources.positiveZ) || _typeof(sources.positiveX) !== _typeof(sources.negativeZ)) {
          throw new Cesium.DeveloperError("this.sources properties must all be the same type.");
        }

        if (typeof sources.positiveX === "string") {
          Cesium.loadCubeMap(context, this._sources).then(function (cubeMap) {
            that._cubeMap = that._cubeMap && that._cubeMap.destroy();
            that._cubeMap = cubeMap;
          });
        } else {
          this._cubeMap = this._cubeMap && this._cubeMap.destroy();
          this._cubeMap = new Cesium.CubeMap({
            context: context,
            source: sources
          });
        }
      }

      var command = this._command;
      command.modelMatrix = Cesium.Transforms.eastNorthUpToFixedFrame(frameState.camera._positionWC);

      if (!Cesium.defined(command.vertexArray) && that._cubeMap) {
        command.uniformMap = {
          u_cubeMap: function u_cubeMap() {
            return that._cubeMap;
          },
          u_rotateMatrix: function u_rotateMatrix() {
            if (!Cesium.defined(Cesium.Matrix4.getRotation)) {
              Cesium.Matrix4.getRotation = Cesium.Matrix4.getMatrix3;
            }

            return Cesium.Matrix4.getRotation(command.modelMatrix, new Cesium.Matrix3());
          }
        };
        var geometry = Cesium.BoxGeometry.createGeometry(Cesium.BoxGeometry.fromDimensions({
          dimensions: new Cesium.Cartesian3(2.0, 2.0, 2.0),
          vertexFormat: Cesium.VertexFormat.POSITION_ONLY
        }));
        var attributeLocations = this._attributeLocations = Cesium.GeometryPipeline.createAttributeLocations(geometry);
        command.vertexArray = Cesium.VertexArray.fromGeometry({
          context: context,
          geometry: geometry,
          attributeLocations: attributeLocations,
          bufferUsage: Cesium.BufferUsage._DRAW
        });
        command.renderState = Cesium.RenderState.fromCache({
          blending: Cesium.BlendingState.ALPHA_BLEND
        });
      }

      if (!Cesium.defined(command.shaderProgram) || this._useHdr !== useHdr) {
        var fs = new Cesium.ShaderSource({
          defines: [useHdr ? "HDR" : ""],
          sources: [this.SkyBoxFS]
        });
        command.shaderProgram = Cesium.ShaderProgram.fromCache({
          context: context,
          vertexShaderSource: this.SkyBoxVS,
          fragmentShaderSource: fs,
          attributeLocations: this._attributeLocations
        });
        this._useHdr = useHdr;
      }

      if (!Cesium.defined(this._cubeMap)) {
        return undefined;
      }

      return command;
    }
  }, {
    key: "isDestroyed",
    value: function isDestroyed() {
      return false;
    }
  }, {
    key: "destroy",
    value: function destroy() {
      var command = this._command;
      command.vertexArray = command.vertexArray && command.vertexArray.destroy();
      command.shaderProgram = command.shaderProgram && command.shaderProgram.destroy();
      this._cubeMap = this._cubeMap && this._cubeMap.destroy();
      return Cesium.destroyObject(this);
    }
  }]);

  return SkyboxGround;
}();

/**
 * 鹰眼图
 * @description 鹰眼图，依赖于二维开源框架leaflet
 * @class
 */

var OverviewMap = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 
   * @param {Object} opt 
   * @param {String} opt.url 地图服务，目前仅支持L.tileLayer
   * @param {Object} opt.style 鹰眼图样式
   * @param {Number} opt.style.height 鹰眼图窗口高度
   * @param {Number} opt.style.width 鹰眼图窗口宽度
   * @param {Number} opt.style.bottom 鹰眼图窗口bottom属性
   * @param {Number} opt.style.right 鹰眼图窗口right属性
   */
  function OverviewMap(viewer, opt) {
    _classCallCheck(this, OverviewMap);

    this.viewer = viewer;
    this.opt = opt || {};
    var defaulteStyle = {
      height: 150,
      width: 200,
      bottom: 30,
      right: 60
    };
    this.style = Object.assign(defaulteStyle, this.opt.style);
    this.rectangle = null;

    if (!opt.url) {
      console.log("缺少地图服务地址，请设置！");
      return;
    }

    this.init();
  }

  _createClass(OverviewMap, [{
    key: "init",
    value: function init() {
      this.mapEle = window.document.createElement("div");
      this.mapEle.setAttribute("id", "map2d");
      this.mapEle.style.height = this.style.height + "px";
      this.mapEle.style.width = this.style.width + "px";
      this.mapEle.style.position = "absolute";
      this.mapEle.style.bottom = this.style.bottom + "px";
      this.mapEle.style.right = this.style.right + "px";
      document.body.appendChild(this.mapEle);
      this.showStyle = {
        color: "#ff7800",
        weight: 1,
        fill: true,
        stroke: true,
        opacity: 1
      };
      this.hideStyle = {
        fill: false,
        opacity: 0
      }; //根据参数创建鹰眼图  

      var map$1 = map('map2d', {
        minZoom: 3,
        maxZoom: 17,
        center: [31.827107, 117.240601],
        zoom: 4,
        zoomControl: false,
        attributionControl: false
      });
      tileLayer(this.opt.url).addTo(map$1);
      this.map = map$1;
      this.viewer.camera.percentageChanged = 0.01;
      this.viewer.camera.changed.addEventListener(this.sceneRenderHandler, this);
      this.sceneRenderHandler();
    }
  }, {
    key: "sceneRenderHandler",
    value: function sceneRenderHandler() {
      var rectangle$1 = this.viewer.camera.computeViewRectangle();
      var extend = {};

      if (rectangle$1) {
        extend.ymin = Cesium.Math.toDegrees(rectangle$1.south);
        extend.ymax = Cesium.Math.toDegrees(rectangle$1.north);
        extend.xmin = Cesium.Math.toDegrees(rectangle$1.west);
        extend.xmax = Cesium.Math.toDegrees(rectangle$1.east);
      } else {
        extend.ymin = -90;
        extend.ymax = 90;
        extend.xmin = -180;
        extend.xmin = 180;
      }

      var corner1 = latLng(extend.ymin, extend.xmin),
          corner2 = latLng(extend.ymax, extend.xmax);
      var bounds = latLngBounds(corner1, corner2);

      if (this.rectangle) {
        this.rectangle.setBounds(bounds);
      } else {
        this.rectangle = rectangle(bounds, this.showStyle).addTo(this.map);
      }

      if (extend.xmin == -180 && extend.xmax == 180 && extend.ymax == 90 && extend.ymin == -90) {
        //整个地球在视域内 
        this.map.setView([0, 0], 0);
        this.rectangle.setStyle(this.hideStyle);
      } else {
        var padBounds = bounds.pad(0.5);
        this.map.fitBounds(padBounds);
        this.rectangle.setStyle(this.showStyle);
      }
    }
    /**
     * 关闭鹰眼图
     */

  }, {
    key: "hide",
    value: function hide() {
      if (this.mapEle) this.mapEle.style.display = "none";
    }
    /**
     * 打开鹰眼图
     */

  }, {
    key: "show",
    value: function show() {
      if (this.map && this.mapEle) this.mapEle.style.display = "block";
    }
    /**
     * 设置鹰眼图窗口内部矩形框样式
     * @param {Object} style 样式属性
     */

  }, {
    key: "setStyle",
    value: function setStyle(style) {
      if (!style) return;
      this.showStyle = style;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.mapEle) {
        document.body.removeChild(this.mapEle);
      }

      this.viewer.camera.changed.removeEventListener(this.sceneRenderHandler, this);
    }
  }]);

  return OverviewMap;
}();

// 点选3dtiles模型
var selectModel = {
  isactivate: false,
  handler: undefined,
  prompt: undefined,
  activate: function activate(viewer, callback) {
    var _this = this;

    if (!viewer) {
      console.log("缺少地图对象--viewer");
      return;
    }

    this.viewer = viewer;

    if (!this.isactivate) {
      this.isactivate = true;
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      if (!this.prompt) this.prompt = new Prompt$1(this.viewer, {});
      this.handler.setInputAction(function (evt) {
        var pick = _this.viewer.scene.pick(evt.position);

        _this.prompt.destroy();

        _this.prompt = undefined;

        if (pick && pick.primitive instanceof Cesium.Cesium3DTileset) {
          if (callback) callback(pick);
        }

        _this.handler.destroy();

        _this.handler = undefined;
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        _this.prompt.update(evt.endPosition, "左键拾取");
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  },
  disable: function disable() {
    if (this.isactivate) {
      this.isactivate = false;

      if (this.handler) {
        this.handler.destroy();
        this.handler = undefined;
      }

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = undefined;
      }
    }
  }
};

// 四周旋转
var viewAround = {
  initView: undefined,
  removeEventHdl: undefined,
  startTime: undefined,
  isStop: false,
  initHeading: undefined,
  viewer: undefined,
  start: function start(viewer, opt) {
    this.viewer = viewer || window.viewer;
    var speed = opt.speed;
    speed = speed || 5;

    if (!this.viewer) {
      console.log("绕点旋转缺少viewer对象");
      return;
    }

    this.end();
    viewer.clock.shouldAnimate = true; //自动播放

    this.isStop = false;
    this.initView = this.getCameraView();
    this.initHeading = this.initView.heading;
    this.startTime = viewer.clock.currentTime;
    var that = this;

    if (!this.removeEventHdl) {
      this.removeEventHdl = viewer.clock.onTick.addEventListener(function () {
        if (that.isStop) return;
        var delTime = Cesium.JulianDate.secondsDifference(viewer.clock.currentTime, that.startTime);
        that.initView.heading = that.initHeading + delTime * speed;
        that.initView.duration = 0;
        that.setCameraView(that.initView);
      });
    }
  },
  end: function end() {
    if (this.removeEventHdl) {
      this.removeEventHdl();
      this.removeEventHdl = undefined;
    }

    this.initView = undefined;
    this.startTime = undefined;
    this.isStop = false;
    this.initHeading = undefined;
  },
  stop: function stop() {
    this.isStop = true;
  },
  goon: function goon() {
    this.initView = this.getCameraView();
    this.startTime = viewer.clock.startTime;
    this.initHeading = this.initView.heading;
    this.isStop = false;
  },
  setSpeed: function setSpeed(speed) {
    this.speed = speed;
  },
  getCameraView: function getCameraView() {
    var camera = this.viewer.camera;
    var position = camera.position;
    var heading = camera.heading;
    var pitch = camera.pitch;
    var roll = camera.roll;
    var lnglat = Cesium.Cartographic.fromCartesian(position);
    return {
      x: Cesium.Math.toDegrees(lnglat.longitude),
      y: Cesium.Math.toDegrees(lnglat.latitude),
      z: lnglat.height,
      heading: Cesium.Math.toDegrees(heading),
      pitch: Cesium.Math.toDegrees(pitch),
      roll: Cesium.Math.toDegrees(roll)
    };
  },
  setCameraView: function setCameraView(obj) {
    if (!obj) return;
    var position = Cesium.Cartesian3.fromDegrees(obj.x, obj.y, obj.z);
    this.viewer.camera.flyTo({
      destination: position,
      orientation: {
        heading: Cesium.Math.toRadians(obj.heading || 0),
        pitch: Cesium.Math.toRadians(obj.pitch || 0),
        roll: Cesium.Math.toRadians(obj.roll || 0)
      },
      duration: obj.duration === undefined ? 3 : obj.duration,
      complete: obj.complete
    });
  }
};

// 绕点旋转
var viewPoint = {
  removeEventLis: null,
  initHeading: 0,
  isStop: false,
  // 是否暂停
  center: null,
  startTime: null,
  viewer: undefined,
  angle: 5,
  start: function start(viewer, opt) {
    this.viewer = viewer || window.viewer;

    if (!this.viewer) {
      console.log("绕点旋转缺少viewer对象");
      return;
    }

    this.end();

    var _ref = opt || {},
        center = _ref.center,
        speed = _ref.speed,
        range = _ref.range,
        pitch = _ref.pitch;

    this.angle = speed || 5;
    range = range || 5000;
    pitch = pitch || -60;
    this.viewer.clock.shouldAnimate = true; //自动播放

    this.center = center;
    if (!this.center) return;

    if (!(this.center instanceof Cesium.Cartesian3)) {
      this.center = Cesium.Cartesian3.fromDegrees(this.center[0], this.center[1], this.center[2] || 0);
    }

    this.startTime = this.viewer.clock.currentTime;
    this.isStop = false;
    var that = this;

    if (!this.removeEventLis) {
      this.removeEventLis = that.viewer.clock.onTick.addEventListener(function () {
        if (that.isStop) return;
        var delTime = Cesium.JulianDate.secondsDifference(that.viewer.clock.currentTime, that.startTime);
        var heading = that.initHeading + delTime * that.angle;
        var hpr = that.getHpr({
          heading: heading,
          pitch: pitch,
          range: range
        });
        console.log(heading);
        that.viewer.camera.lookAt(that.center, hpr);
      });
    }
  },
  stop: function stop() {
    this.isStop = true;
  },
  goon: function goon() {
    this.startTime = this.viewer.clock.startTime;
    this.isStop = false;
  },
  end: function end() {
    if (this.removeEventLis) {
      this.removeEventLis();
      this.removeEventLis = null;
    }

    this.initHeading = 0;
    this.isStop = false;
    this.center = null;
    this.startTime = null;
    that.viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
  },
  setSpeed: function setSpeed(speed) {
    this.angle = speed;
  },
  getHpr: function getHpr(opt) {
    var heading = Cesium.Math.toRadians(opt.heading || 0);
    var pitch = Cesium.Math.toRadians(opt.pitch || -60);
    var range = opt.range || 5000;
    return new Cesium.HeadingPitchRange(heading, pitch, range);
  },
  setCenter: function setCenter(center) {
    this.center = center;
  }
};

var worldRotate = {
  start: function start(viewer, obj, callback) {
    //传入所需定位的经纬度 及旋转的速度 旋转的圈数
    if (!obj.x || !obj.y) {
      console.log("设定地球旋转时，并未传入经纬度！");
      return;
    }

    var v = obj.v || 1;
    var i = 0;
    var q = obj.q || 2;
    var x = obj.x;
    var y = obj.y;
    var z = obj.z;
    var interVal = window.setInterval(function () {
      x = x + v;

      if (x >= 179) {
        x = -180;
        i++;
      }

      viewer.scene.camera.setView({
        destination: new Cesium.Cartesian3.fromDegrees(x, y, z || 20000000)
      });

      if (i == q) {
        //此处有瑕疵  未修改
        clearInterval(interVal);
        callback();
      }
    }, 16);
  }
};

/**
 * @class
 * @description 模型单方向裁剪
 */
var TilesetClip = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象
   * @param {Object} opt 
   * @param {Cesium.Cesium3DTileset} opt.tileset 3dtiles模型
   * @param {Cesium.Cartesian3} opt.dir 裁剪方向（按某个方向裁剪）
   * @param {Number} opt.distance 裁剪距离
   */
  function TilesetClip(viewer, opt) {
    _classCallCheck(this, TilesetClip);

    this.viewer = viewer;
    this.opt = opt || {};
    this.tileset = opt.tileset;
    if (!this.viewer || !this.tileset) return;
    this.planeEntities = [];

    if (opt.dir) {
      // 按某个方向裁剪
      this.dir = Cesium.Cartesian3.normalize(opt.dir.clone(), new Cesium.Cartesian3());
    } else if (opt.angle != undefined) {
      // 按某个角度裁剪
      var defaultDir = new Cesium.Cartesian3(0.0, 1.0, 0.0);
      defaultDir = this.angle2dir(Number(this.opt.angle), defaultDir.clone());
      this.dir = Cesium.Cartesian3.normalize(defaultDir.clone(), new Cesium.Cartesian3());
      this.reverse = this.computeReverse(this.dir.clone());
    } else {
      // 竖直方向裁剪
      this.dir = Cesium.Cartesian3.normalize(new Cesium.Cartesian3(0.0, 0.0, -1.0), new Cesium.Cartesian3());
      this.isrelyx = true;
    }

    this.distance = opt.distance || 0;
    this.selectedPlane = undefined;
    this.downHandler = undefined;
    this.upHandler = undefined;
    this.moveHandler = undefined;
    this.scale = this.opt.scale || [1, 1];
  }
  /**
   * 开始裁剪
   */


  _createClass(TilesetClip, [{
    key: "start",
    value: function start() {
      this.createClippingPlanes();
      this.bindHandler();
      this.viewer.scene.render();
    }
  }, {
    key: "end",
    value: function end() {
      if (this.downHandler) {
        this.downHandler.destroy();
        this.downHandler = undefined;
      }

      if (this.upHandler) {
        this.upHandler.destroy();
        this.upHandler = undefined;
      }

      if (this.moveHandler) {
        this.moveHandler.destroy();
        this.moveHandler = undefined;
      }

      this.planeEntities.forEach(function (ent) {
        ent.show = false;
      });
      this.selectedPlane = undefined;
      this.distance = 0;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.downHandler) {
        this.downHandler.destroy();
        this.downHandler = undefined;
      }

      if (this.upHandler) {
        this.upHandler.destroy();
        this.upHandler = undefined;
      }

      if (this.moveHandler) {
        this.moveHandler.destroy();
        this.moveHandler = undefined;
      }

      this.selectedPlane = undefined;
      var that = this;
      this.planeEntities.forEach(function (ent) {
        that.viewer.entities.remove(ent);
      });
      this.planeEntities = [];
      this.tileset.clippingPlanes = new Cesium.ClippingPlaneCollection();
    } // 构建模型切割面

  }, {
    key: "createClippingPlanes",
    value: function createClippingPlanes() {
      var clippingPlanes = new Cesium.ClippingPlaneCollection({
        planes: [new Cesium.ClippingPlane(this.dir, 0.0)],
        edgeWidth: 1.0
      });
      this.tileset.clippingPlanes = clippingPlanes;
      var that = this;

      for (var i = 0; i < clippingPlanes.length; ++i) {
        var clipplane = clippingPlanes.get(i);
        var planeEntity = this.viewer.entities.add({
          position: this.tileset.boundingSphere.center,
          plane: {
            dimensions: new Cesium.Cartesian2(this.tileset.boundingSphere.radius * 1, this.tileset.boundingSphere.radius * 1),
            material: Cesium.Color.WHITE.withAlpha(.3),
            plane: new Cesium.CallbackProperty(that.createPlaneUpdateFunction(clipplane), false),
            outline: true,
            outlineColor: Cesium.Color.WHITE.withAlpha(.3)
          }
        });
        this.planeEntities.push(planeEntity);
      }
    } // 绑定事件

  }, {
    key: "bindHandler",
    value: function bindHandler() {
      var that = this;
      if (!this.downHandler) this.downHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      if (!this.upHandler) this.upHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      if (!this.moveHandler) this.moveHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.downHandler.setInputAction(function (movement) {
        var pickedObject = that.viewer.scene.pick(movement.position);

        if (Cesium.defined(pickedObject) && Cesium.defined(pickedObject.id) && Cesium.defined(pickedObject.id.plane)) {
          that.selectedPlane = pickedObject.id.plane;
          that.selectedPlane.material = Cesium.Color.WHITE.withAlpha(0.1);
          that.selectedPlane.outlineColor = Cesium.Color.WHITE;
          that.viewer.scene.screenSpaceCameraController.enableInputs = false; // 禁止当前操作外的其它操作 防止操作的混淆
        }
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
      this.upHandler.setInputAction(function () {
        if (Cesium.defined(that.selectedPlane)) {
          that.selectedPlane.material = Cesium.Color.WHITE.withAlpha(0.1);
          that.selectedPlane.outlineColor = Cesium.Color.WHITE;
          that.selectedPlane = undefined;
        }

        that.viewer.scene.screenSpaceCameraController.enableInputs = true;
      }, Cesium.ScreenSpaceEventType.LEFT_UP);
      this.moveHandler.setInputAction(function (movement) {
        if (Cesium.defined(that.selectedPlane)) {
          var delta;

          if (that.isrelyx) {
            delta = -1 * (movement.endPosition.y - movement.startPosition.y);
          } else {
            delta = that.computeDis(movement);
          }

          that.distance += delta;
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "createPlaneUpdateFunction",
    value: function createPlaneUpdateFunction(plane) {
      var that = this;
      return function () {
        plane.distance = that.distance;
        return plane;
      };
    }
  }, {
    key: "angle2dir",
    value: function angle2dir(angle, olddir) {
      var m = Cesium.Matrix3.fromRotationZ(Cesium.Math.toRadians(angle));
      var dir = Cesium.Matrix3.multiplyByVector(m, olddir, new Cesium.Cartesian3());
      dir = Cesium.Cartesian3.normalize(dir, new Cesium.Cartesian3());
      return dir;
    } // 判断是否要对计算进行反转

  }, {
    key: "computeReverse",
    value: function computeReverse(dir) {
      var res = Cesium.Cartesian3.dot(new Cesium.Cartesian3(0, 1, 0), dir.clone());
      res = res < 0 ? 1 : 1;
      return res;
    }
  }, {
    key: "computeDis",
    value: function computeDis(movement) {
      var that = this;
      movement.startPosition.x = movement.startPosition.x / this.scale[0];
      movement.startPosition.y = movement.startPosition.y / this.scale[1];
      movement.endPosition.x = movement.endPosition.x / this.scale[0];
      movement.endPosition.y = movement.endPosition.y / this.scale[1];
      var ray1 = that.viewer.camera.getPickRay(movement.startPosition);
      var startP = that.viewer.scene.globe.pick(ray1, that.viewer.scene);
      var ray2 = that.viewer.camera.getPickRay(movement.endPosition);
      var endP = that.viewer.scene.globe.pick(ray2, that.viewer.scene);
      var movedir = Cesium.Cartesian3.subtract(endP, startP, new Cesium.Cartesian3());
      var res = Cesium.Cartesian3.dot(movedir, that.dir);
      return res;
    }
  }]);

  return TilesetClip;
}();

// 模型点光源
var PointLight = /*#__PURE__*/function () {
  function PointLight(tileset, opt) {
    _classCallCheck(this, PointLight);

    this.opt = opt || {};
    this.tileset = tileset;
    /**
     * 支持多光源 
     * [
     *  {
     *      id : "" , // 可根据此id来进行单个光源的控制
     *      position : Ceisum.Cartesian3 ,// 光源位置
     *      color : Cesium.Color, // 颜色
     *      length : 1500 , // 光源边界距离
     *      intensity : 50 , // 光照强度
     *  }
     * ]
     */

    this.lights = opt.lights || [];
    if (this.lights.length < 1) return;
    this.customShader = this.getUnformsShader();
    this.tileset.customShader = this.customShader;
  }

  _createClass(PointLight, [{
    key: "getUnformsShader",
    value: function getUnformsShader() {
      var uniforms = {};
      var inputText = "";
      var ids = [];

      for (var i = 0; i < this.lights.length; i++) {
        var item = this.lights[i];
        var id = item.id,
            position = item.position,
            targetPosition = item.targetPosition,
            color = item.color,
            length = item.length,
            innerRange = item.innerRange,
            outRange = item.outRange,
            intensity = item.intensity;
        if (!position) continue;
        id = id || new Date().getTime() + "-" + Math.ceil(Math.random() * 1000);
        ids.push(id);
        uniforms["u_lightPosition_".concat(id)] = {
          type: Cesium.UniformType.VEC3,
          value: position
        };
        uniforms["u_lightColor_".concat(id)] = {
          type: Cesium.UniformType.VEC4,
          value: color || Cesium.Color.RED
        };
        uniforms["u_length_".concat(id)] = {
          type: Cesium.UniformType.FLOAT,
          value: length || 500
        };
        uniforms["u_intensity_".concat(id)] = {
          type: Cesium.UniformType.FLOAT,
          value: intensity || 10
        };
        inputText += "\n                vec4 fcolor_".concat(id, " = pointLight(\n                    u_lightColor_").concat(id, ",\n                    u_lightPosition_").concat(id, ",\n                    positionWC,\n                    positionEC,\n                    normalEC,\n                    pbrParameters,\n                    u_intensity_").concat(id, ",\n                    u_length_").concat(id, "\n                );\n            ");
      } // 颜色混合


      var mixText = "";

      if (ids.length > 1) {
        mixText = "\n            vec3 finalColor = mix(\n                fcolor_".concat(ids[0], ".rgb, \n                fcolor_").concat(ids[1], ".rgb,\n                0.5);\n            "); // 光源混合

        ids.forEach(function (id, index) {
          if (index > 1) {
            mixText += "\n                    finalColor = mix(\n                            finalColor.rgb,\n                            fcolor_".concat(id, ".rgb,\n                            0.5\n                        );\n                ");
          }
        });
      } else {
        mixText = "vec3 finalColor = fcolor_".concat(ids[0], ".rgb;");
      }

      var fragmentShaderText = "\n            vec4 pointLight(\n                vec4 lightColor,\n                vec3 lightPosition,\n                vec3 positionWC,\n                vec3 positionEC,\n                vec3 normalEC,\n                czm_pbrParameters pbrParameters,\n                float intensity,\n                float plength\n                ){\n                    // \u5149\u6E90-\u7247\u5143\u5411\u91CF\n                    vec3 lightToPoint = (czm_view * vec4(lightPosition,1.0)).xyz - positionEC;\n                    vec3 lightToPoint_dir = normalize(lightToPoint);\n                    // \u8BA1\u7B97\u5149\u6E90\u4F4D\u7F6E-\u504F\u8FDC\u4F4D\u7F6E\u7EC4\u6210\u7684\u5411\u91CF \u4E0E \u5149\u7EBF\u5411\u91CF\u7684 \u70B9\u4E58\uFF0C\u5373\u5F53\u524D\u7247\u5143\u7684\u989C\u8272\n                    // \u989C\u8272\u63D2\u503C\n                    float inLight = 1.0;;\n                    // \u5224\u65AD\u70B9\u5149\u6E90\u548C\u7247\u5143\u7684\u4F4D\u7F6E\u5173\u7CFB 0-\u5782\u76F4 1-\u6B63\u4E0A\u65B9\n                    float light = inLight * clamp(dot(normalEC,lightToPoint_dir),0.0,1.0); //\u6F2B\u53CD\u5C04\n                    // \u5149\u7EBF\u8870\u51CF\n                    vec3 light1Dir = positionWC - lightPosition;\n                    float lDistance = 1.0;\n                    // \u6839\u636E\u4E0E\u5149\u6E90\u8DDD\u79BB \u8BA1\u7B97\u5F3A\u5EA6\n                    lDistance = 1.0 - min( ( length( light1Dir ) / plength ), 1.0 );\n                    // \u8D85\u51FA\u8303\u56F4 \u53D8\u4E3A\u6A21\u578B\u9ED1\u8272  \u8303\u56F4\u5185 \u5149\u5F3A\u9012\u51CF\n                    if(lDistance < 0.00001 ) {\n                        return vec4(0.0);\n                    }else{\n                        vec3 diffuseColor = pbrParameters.diffuseColor;\n                        // \u8FB9\u7F18\u53D8\u9ED1 \u5982\u679C\u4E0D\u60F3\u53D8\u9ED1 \u5C31\u53BB\u6389light\n                        diffuseColor *= light * lightColor.rgb; \n                        diffuseColor *=  lDistance * intensity;\n                        return vec4(diffuseColor,lDistance);\n                    }\n                }\n\n            void fragmentMain(FragmentInput fsInput, inout czm_modelMaterial material){\n                    material.diffuse = vec3(1.0,1.0,1.0); // \u8BBE\u7F6E\u6A21\u578B\u57FA\u5730\u8272\u4E3A\u767D\u8272 \u9632\u6B62\u51FA\u73B0\u989C\u8272\u504F\u5DEE\n                    vec3 positionWC = fsInput.attributes.positionWC;\n                    vec3 normalEC = fsInput.attributes.normalEC;\n                    vec3 positionEC = fsInput.attributes.positionEC;\n                    czm_pbrParameters pbrParameters;\n                    pbrParameters.diffuseColor = material.diffuse;\n                    // \u8BBE\u7F6E\u53CD\u5C04\u7387\n                    pbrParameters.f0 = vec3(0.5);\n                    // \u8BBE\u7F6E\u7C97\u7CD9\u7A0B\u5EA6\n                    pbrParameters.roughness = 1.0;\n                    // \u8BBE\u7F6E\u7167\u660E\u53C2\u6570\n                    ".concat(inputText, ";\n                    ").concat(mixText, ";\n                    material.diffuse *= finalColor;\n                }\n            ");
      var customShader = new Cesium.CustomShader({
        lightingModel: Cesium.LightingModel.UNLIT,
        uniforms: uniforms,
        fragmentShaderText: fragmentShaderText
      });
      return customShader;
    }
  }, {
    key: "destroy",
    value: function destroy() {
      this.tileset.customShader = undefined;
    }
  }]);

  return PointLight;
}();

// 三维模型裁剪
var TilesetCut = /*#__PURE__*/function () {
  function TilesetCut(tileset, opt) {
    _classCallCheck(this, TilesetCut);

    if (!tileset) {
      console.log("缺少模型");
      return;
    }

    this.tileset = tileset;
    this.opt = opt || {};
    /**
     * @property {Boolean} iscutOutter 是否为外部裁剪，默认为内部裁剪
     */

    this._iscutOutter = this.opt.iscutOutter; // 是否为外部裁剪  默认为内部裁剪

    this.cutRegions = []; // 当前裁剪面数组对象

    /* this.modelMatrix = new Cesium.Matrix4(); // 世界坐标系--》模型坐标系
    Cesium.Matrix4.inverseTransformation(this.tileset.root.computedTransform, this.modelMatrix) */
    // 建立模型中心点坐标系

    var center = this.tileset.boundingSphere.center;
    var enuMtx4 = Cesium.Transforms.eastNorthUpToFixedFrame(center);
    this.modelMatrix = Cesium.Matrix4.inverse(enuMtx4, new Cesium.Matrix4());
    this.canvas = undefined;
  }

  _createClass(TilesetCut, [{
    key: "iscutOutter",
    get: function get() {
      return this._iscutOutter;
    },
    set: function set(val) {
      this._iscutOutter = val;
      this.updateShader();
    }
    /**
     * 添加裁剪面
     * @param {Object} attr 参数
     * @param {Cesium.Cartesian3[]} attr.positions 压平面坐标
     * @param {Number} attr.id 唯一标识
     */

  }, {
    key: "addRegion",
    value: function addRegion(attr) {
      var _ref = attr || {},
          positions = _ref.positions,
          id = _ref.id;

      if (!id) id = new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0);

      if (!positions || positions.length < 3) {
        console.log("缺少裁剪面坐标");
        return;
      }

      var index = this.cutRegions.findIndex(function (item) {
        return item.id === id;
      });

      if (index == -1) {
        this.cutRegions.push({
          id: id,
          positions: positions
        });
      } else {
        this.cutRegions[index].positions = positions;
      }

      this.updateShader();
    }
    /**
     * 移除裁剪面
     * @param {String} id 
     */

  }, {
    key: "removeRegionById",
    value: function removeRegionById(id) {
      if (id) {
        // 表示移除所有的裁剪面
        var index = this.cutRegions.findIndex(function (item) {
          return item.id === id;
        });
        if (index != -1) this.cutRegions.splice(index, 1);
      } else {
        // 表示移除单个的裁剪面
        this.cutRegions = [];
      }

      this.updateShader();
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      this.tileset.customShader = undefined;
    }
    /**
     * 修改模型着色器
     */

  }, {
    key: "updateShader",
    value: function updateShader() {
      var _this = this;// 定义着色器中裁剪函数

      var fs_textureMapRect = "\n            vec4 textureMapRect(vec4 rect, sampler2D map, vec2 xy) {\n                // \u5224\u65AD\u5F53\u524D\u56FE\u5143\u5750\u6807\u548C\u591A\u8FB9\u5F62\u5173\u7CFB \u5982\u679C\u5728\u591A\u8FB9\u5F62\u5185 \u8FDB\u884C\u7EB9\u7D20\u62FE\u53D6\n                if (xy.x >= rect.x && xy.x <= rect.z && xy.y >= rect.y && xy.y <= rect.w) {\n                    float w = rect.z - rect.x;\n                    float h = rect.w - rect.y;\n                    float s = (xy.x - rect.x) / w;\n                    float t = (xy.y - rect.y) / h;\n                    vec4 color = texture(map, vec2(s, 1.0 - t));\n                    return color;\n                }\n                return vec4(1.0);\n            }\n        ";
      var allUniforms = {
        u_inverseModel: {
          type: Cesium.UniformType.MAT4,
          value: this.modelMatrix.clone()
        },
        u_unionCutRegions: {
          type: Cesium.UniformType.BOOL,
          value: this._iscutOutter
        }
      }; // 构建多区域着色器

      var fs = "";
      this.cutRegions.forEach(function (element) {
        var uniforms = _this.createUniforms(element.positions, element.id);

        allUniforms = Cesium.combine(allUniforms, uniforms);
        fs += "\n                vec4 color_".concat(element.id, " = textureMapRect(u_rect_").concat(element.id, ", u_map_").concat(element.id, ", xy);\n                cutColor *= color_").concat(element.id, ";\n            ");
      });
      fs += "\n            if (u_unionCutRegions) {\n                material.diffuse *= (vec3(1.0) - cutColor.rgb);\n            } else {\n                material.diffuse *= cutColor.rgb;\n            }\n            if (material.diffuse.r <= 0.0001 && material.diffuse.g <= 0.0001 && material.diffuse.b <= 0.0001) {\n                discard;\n            }\n        ";
      this.tileset.customShader = new Cesium.CustomShader({
        uniforms: allUniforms,
        fragmentShaderText: " \n            ".concat(fs_textureMapRect, "\n            void fragmentMain(FragmentInput fsInput, inout czm_modelMaterial material) {\n                vec4 positionMC = u_inverseModel * vec4(fsInput.attributes.positionWC, 1.0);\n                vec2 xy = positionMC.xy;\n                vec4 cutColor = vec4(1.0);\n                ").concat(fs, "\n            }")
      });
    }
    /**
     * 根据坐标创建片元着色器
     * @param {Cartesian3[]} positions 
     * @param {String} id 
     */

  }, {
    key: "createUniforms",
    value: function createUniforms(positions, id) {
      var _this2 = this;

      if (!positions || positions.length < 3) {
        console.log("缺少裁剪面坐标");
        return;
      }

      id = id || Math.ceil(Math.random() * 100000) + '_' + Math.ceil(Math.random() * 100000); // 根据世界坐标范围计算相对模型坐标范围

      var xs = [],
          ys = [],
          zs = []; // 计算模型坐标系下坐标

      var modelPoints = positions.map(function (p) {
        var point = Cesium.Matrix4.multiplyByPoint(_this2.modelMatrix, p, new Cesium.Cartesian3());
        xs.push(point.x);
        ys.push(point.y);
        zs.push(point.z);
        return point;
      }); // 计算当前裁剪面边界范围（模型坐标系下）

      var rect = new Cesium.Cartesian4(Math.min.apply(null, xs), Math.min.apply(null, ys), Math.max.apply(null, xs), Math.max.apply(null, ys));
      var canvas = document.createElement('canvas');
      canvas.width = 1024;
      canvas.height = 1024;
      var width = rect.z - rect.x;
      var height = rect.w - rect.y;
      var ctx = canvas.getContext('2d');
      ctx.fillStyle = '#fff'; // 设置整体背景为白色

      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.beginPath();
      ctx.moveTo(canvas.width * (modelPoints[0].x - rect.x) / width, canvas.height * (modelPoints[0].y - rect.y) / height);

      for (var i = 1; i < modelPoints.length; i++) {
        ctx.lineTo(canvas.width * (modelPoints[i].x - rect.x) / width, canvas.height * (modelPoints[i].y - rect.y) / height);
      }

      ctx.closePath();
      ctx.fillStyle = '#000'; // 根据填充的黑色来裁剪模型

      ctx.fill();
      this.canvas = canvas;
      var uniforms = {};
      uniforms["u_rect_".concat(id)] = {
        type: Cesium.UniformType.VEC4,
        value: rect
      };
      uniforms["u_map_".concat(id)] = {
        type: Cesium.UniformType.SAMPLER_2D,
        value: new Cesium.TextureUniform({
          url: canvas.toDataURL()
        }),
        minificationFilter: Cesium.TextureMinificationFilter.LINEAR,
        magnificationFilter: Cesium.TextureMagnificationFilter.LINEAR
      };
      return uniforms;
    }
  }]);

  return TilesetCut;
}();

/**
 * @class
 * @description 3dtiles模型压平
 */
var Flat = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Cesium3DTileset} tileset 三维模型
   * @param {Object} opt 
   * @param {Number} opt.flatHeight 压平高度 
   */
  function Flat(tileset, opt) {
    _classCallCheck(this, Flat);

    _defineProperty(this, "getUniqueArray", function (arr) {
      return arr.filter(function (item, index, arr) {
        //当前元素，在原始数组中的第一个索引==当前索引值，否则返回当前元素
        return arr.indexOf(item, 0) === index;
      });
    });

    if (!tileset) return;
    this.tileset = tileset;
    this.opt = opt || {};
    this.flatHeight = this.opt.flatHeight || 0;
    this.center = tileset.boundingSphere.center.clone();
    this.matrix = Cesium.Transforms.eastNorthUpToFixedFrame(this.center.clone());
    this.localMatrix = Cesium.Matrix4.inverse(this.matrix, new Cesium.Matrix4()); // 多面的坐标数组

    this.regionList = []; // 多个面坐标转为局部模型坐标

    this.localPositionsArr = [];
  }
  /**
   * 添加压平面
   * @param {Object} attr 参数
   * @param {Cesium.Cartesian3[]} attr.positions 压平面坐标
   * @param {Number} attr.height 压平深度，当前不支持单独设置
   * @param {Number} attr.id 唯一标识
   */


  _createClass(Flat, [{
    key: "addRegion",
    value: function addRegion(attr) {
      var _this = this;

      var _ref = attr || {},
          positions = _ref.positions,
          height = _ref.height,
          id = _ref.id; // this.flatHeight = height;


      if (!id) id = new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0);
      this.regionList.push(attr);

      for (var i = 0; i < this.regionList.length; i++) {
        var item = this.regionList[i];
        var _positions = item.positions;
        var localCoor = this.cartesiansToLocal(_positions);
        this.localPositionsArr.push(localCoor);
      }

      var funstr = this.getIsinPolygonFun(this.localPositionsArr);
      var str = "";

      var _loop = function _loop(_i) {
        var coors = _this.localPositionsArr[_i];
        var n = coors.length;
        var instr = "";
        coors.forEach(function (coordinate, index) {
          instr += "points_".concat(n, "[").concat(index, "] = vec2(").concat(coordinate[0], ", ").concat(coordinate[1], ");\n");
        });
        str += "\n                ".concat(instr, "\n                if(isPointInPolygon_").concat(n, "(position2D)){\n                    vec4 tileset_local_position_transformed = vec4(tileset_local_position.x, tileset_local_position.y, ground_z, 1.0);\n                    vec4 model_local_position_transformed = czm_inverseModel * u_tileset_localToWorldMatrix * tileset_local_position_transformed;\n                    vsOutput.positionMC.xy = model_local_position_transformed.xy;\n                    vsOutput.positionMC.z = model_local_position_transformed.z+ modelMC.z*0.002;\n                    return;\n                }");
      };

      for (var _i = 0; _i < this.localPositionsArr.length; _i++) {
        _loop(_i);
      }

      this.updateShader(funstr, str);
    }
    /**
     * 根据id删除压平的面
     * @param {String} id 唯一标识
     */

  }, {
    key: "removeRegionById",
    value: function removeRegionById(id) {
      var _this2 = this;

      if (!id) return;
      this.regionList = this.regionList.filter(function (attr) {
        return attr.id != id;
      });
      this.localPositionsArr = [];

      for (var i = 0; i < this.regionList.length; i++) {
        var item = this.regionList[i];
        var positions = item.positions;
        var localCoor = this.cartesiansToLocal(positions);
        this.localPositionsArr.push(localCoor);
      }

      var funstr = this.getIsinPolygonFun(this.localPositionsArr);
      var str = "";

      var _loop2 = function _loop2(_i2) {
        var coors = _this2.localPositionsArr[_i2];
        var n = coors.length;
        var instr = "";
        coors.forEach(function (coordinate, index) {
          instr += "points_".concat(n, "[").concat(index, "] = vec2(").concat(coordinate[0], ", ").concat(coordinate[1], ");\n");
        });
        str += "\n                ".concat(instr, "\n                if(isPointInPolygon_").concat(n, "(position2D)){\n                    vec4 tileset_local_position_transformed = vec4(tileset_local_position.x, tileset_local_position.y, ground_z, 1.0);\n                    vec4 model_local_position_transformed = czm_inverseModel * u_tileset_localToWorldMatrix * tileset_local_position_transformed;\n                    vsOutput.positionMC.xy = model_local_position_transformed.xy;\n                    vsOutput.positionMC.z = model_local_position_transformed.z+ modelMC.z*0.002;\n                    return;\n                }");
      };

      for (var _i2 = 0; _i2 < this.localPositionsArr.length; _i2++) {
        _loop2(_i2);
      }

      this.updateShader(funstr, str);
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      this.tileset.customShader = undefined;
    }
    /**
     * 根据数组长度，构建 判断点是否在面内 的压平函数
     */

  }, {
    key: "getIsinPolygonFun",
    value: function getIsinPolygonFun(polygons) {
      var pmap = polygons.map(function (polygon) {
        return polygon.length;
      });
      var uniqueArray = this.getUniqueArray(pmap);
      var str = "";
      uniqueArray.forEach(function (length) {
        str += "\n                vec2 points_".concat(length, "[").concat(length, "];\n                bool isPointInPolygon_").concat(length, "(vec2 point){\n                int nCross = 0; // \u4EA4\u70B9\u6570\n                const int n = ").concat(length, "; \n                for(int i = 0; i < n; i++){\n                    vec2 p1 = points_").concat(length, "[i];\n                    vec2 p2 = points_").concat(length, "[int(mod(float(i+1),float(n)))];\n                    if(p1[1] == p2[1]){\n                        continue;\n                    }\n                    if(point[1] < min(p1[1], p2[1])){\n                        continue;\n                    }\n                    if(point[1] >= max(p1[1], p2[1])){\n                        continue;\n                    }\n                    float x = p1[0] + ((point[1] - p1[1]) * (p2[0] - p1[0])) / (p2[1] - p1[1]);\n                    if(x > point[0]){\n                     nCross++;\n                    }\n                }\n\n                return int(mod(float(nCross), float(2))) == 1;\n                }\n            ");
      });
      return str;
    }
  }, {
    key: "updateShader",
    value: function updateShader(vtx1, vtx2) {
      var flatCustomShader = new Cesium.CustomShader({
        uniforms: {
          u_tileset_localToWorldMatrix: {
            type: Cesium.UniformType.MAT4,
            value: this.matrix
          },
          u_tileset_worldToLocalMatrix: {
            type: Cesium.UniformType.MAT4,
            value: this.localMatrix
          },
          u_flatHeight: {
            type: Cesium.UniformType.FLOAT,
            value: this.flatHeight
          }
        },
        vertexShaderText: "\n            // \u6240\u6709isPointInPolygon\u51FD\u6570\n            ".concat(vtx1, "\n            void vertexMain(VertexInput vsInput, inout czm_modelVertexOutput vsOutput){\n                vec3 modelMC = vsInput.attributes.positionMC;\n                vec4 model_local_position = vec4(modelMC.x, modelMC.y, modelMC.z, 1.0);\n                vec4 tileset_local_position = u_tileset_worldToLocalMatrix * czm_model * model_local_position;\n                vec2 position2D = vec2(tileset_local_position.x,tileset_local_position.y);\n                float ground_z = 0.0 + u_flatHeight;\n                // \u591A\u4E2A\u591A\u8FB9\u5F62\u533A\u57DF\n                ").concat(vtx2, "\n            }")
      });
      this.tileset.customShader = flatCustomShader;
    } // 数组去重，不能处理嵌套的数组

  }, {
    key: "cartesiansToLocal",
    value: // 世界坐标转数组局部坐标
    function cartesiansToLocal(positions) {
      var arr = [];

      for (var i = 0; i < positions.length; i++) {
        var position = positions[i];
        var localp = Cesium.Matrix4.multiplyByPoint(this.localMatrix, position.clone(), new Cesium.Cartesian3());
        arr.push([localp.x, localp.y]);
      }

      return arr;
    }
  }]);

  return Flat;
}();

/**
 * 日照分析
 * @class
 */
var Sunshine = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础参数
   * @param {Date|Cesium.JulianDate} opt.startTime 开始时间
   * @param {Date|Cesium.JulianDate} opt.endTime 结束时间
   * @param {Number} [opt.multiplier=1] 播放速度
   */
  function Sunshine(viewer, opt) {
    _classCallCheck(this, Sunshine);

    this.viewer = viewer;
    this.opt = opt || {};
    /**
     * @property {Cesium.JulianDate} _startTime 开始时间
     */

    this._startTime = opt.startTime || Cesium.JulianDate.fromDate(new Date().setHours(8), new Cesium.JulianDate());
    if (this._startTime instanceof Date) this._startTime = Cesium.JulianDate.fromDate(this._startTime, new Cesium.JulianDate());
    /**
     * @property {Cesium.JulianDate} _endTime 结束时间
     */

    this._endTime = opt.endTime;
    if (this._endTime instanceof Date) this._endTime = Cesium.JulianDate.fromDate(this._endTime, new Cesium.JulianDate());
    this.oldShouldAnimate = this.viewer.clock.shouldAnimate;
    this.multiplier = opt.multiplier || 60;
    this.oldenableLighting = this.viewer.scene.globe.enableLighting;
    this.oldshadows = this.viewer.shadows;
  }
  /**
   * 开始
   */


  _createClass(Sunshine, [{
    key: "start",
    value: function start() {
      this.viewer.clock.currentTime = this._startTime.clone();
      this.viewer.clock.startTime = this._startTime.clone();
      this.viewer.clock.shouldAnimate = true;
      this.viewer.clock.multiplier = this.multiplier;
      this.viewer.scene.globe.enableLighting = true;
      this.viewer.shadows = true;
      this.viewer.clock.clockRange = Cesium.ClockRange.LOOP_STOP;
      if (this._endTime) this.viewer.clock.endTime = this._endTime.clone();
    }
    /**
     * 结束
     */

  }, {
    key: "end",
    value: function end() {
      this.viewer.clock.clockRange = Cesium.ClockRange.UNBOUNDED;
      this.viewer.clock.shouldAnimate = this.oldShouldAnimate;
      this.viewer.clock.multiplier = 1;
      this.viewer.scene.globe.enableLighting = this.oldenableLighting;
      this.viewer.shadows = this.oldshadows;
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      this.viewer.clock.clockRange = Cesium.ClockRange.UNBOUNDED;
      this.viewer.clock.shouldAnimate = this.oldShouldAnimate;
      this.viewer.clock.multiplier = 1;
      this.viewer.scene.globe.enableLighting = this.oldenableLighting;
    }
    /**
     * 暂停/继续
     */

  }, {
    key: "pause",
    value: function pause() {
      this.viewer.clock.shouldAnimate = !this.viewer.clock.shouldAnimate;
    }
  }, {
    key: "startTime",
    get: function get() {
      return this._startTime;
    },
    set: function set(time) {
      if (!time) return;

      if (this._startTime instanceof Date) {
        this._startTime = Cesium.JulianDate.fromDate(this._startTime, new Cesium.JulianDate());
      } else {
        this._startTime = time.clone();
      }

      this.start();
    }
  }, {
    key: "endTime",
    get: function get() {
      return this._endTime;
    },
    set: function set(time) {
      if (!time) return;

      if (this._endTime instanceof Date) {
        this._endTime = Cesium.JulianDate.fromDate(this._startTime, new Cesium.JulianDate());
      } else {
        this._endTime = time.clone();
      }

      this.start();
    }
  }]);

  return Sunshine;
}();

/**
 * 限高分析
 * @class
 */
var LimitHeight = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础参数
   * @param {Cesium.Cartesian3[]} opt.positions 限制范围
   * @param {Number} opt.bottomHeight 最小高度
   * @param {Number} [opt.topHeight=Number.MAX_VALUE] 最大高度
   * @param {String} [opt.color='#ff0000'] 颜色
   * @param {Number} [opt.alpha=0.8] 颜色透明度
   */
  function LimitHeight(viewer, opt) {
    _classCallCheck(this, LimitHeight);

    this.viewer = viewer;
    /**
     * @property {Cesium.Cartesian3[]} positions 限制范围
     */

    this.positions = opt.positions;
    /**
     * @property {Number} bottomHeight 最小高度
     */

    this.bottomHeight = Number(opt.bottomHeight);
    if (this.bottomHeight == undefined) return;
    /**
     * @property {Number} topHeight 最大高度
     */

    this.topHeight = Number(opt.topHeight) || Number.MAX_VALUE;
    var icolor = opt.color || "#ff0000";
    this.color = icolor instanceof Cesium.Color ? icolor : Cesium.Color.fromCssColorString(icolor);
    this.colorAlpha = opt.alpha || 0.8;
    this.primitive = undefined; // this.extrudedHeight = this.topHeight - this.bottomHeight;

    this.init();
  }
  /**
   * 初始化
   */


  _createClass(LimitHeight, [{
    key: "init",
    value: function init() {
      var polygonInstance = new Cesium.GeometryInstance({
        geometry: new Cesium.PolygonGeometry({
          polygonHierarchy: new Cesium.PolygonHierarchy(this.positions),
          height: this.bottomHeight,
          extrudedHeight: this.topHeight
        }),
        attributes: {
          color: Cesium.ColorGeometryInstanceAttribute.fromColor(this.color.withAlpha(this.colorAlpha))
        }
      });
      this.primitive = this.viewer.scene.primitives.add(new Cesium.ClassificationPrimitive({
        geometryInstances: polygonInstance,
        releaseGeometryInstances: false,
        classificationType: Cesium.ClassificationType.CESIUM_3D_TILE
      }));
      /* 
      let that = this;
      this.primitive.readyPromise.then((primitive) => {
          this.setHeight();
      }); */
    }
    /**
     * 
     * @param {Number} h 限制高度 
     */

  }, {
    key: "setHeight",
    value: function setHeight(h) {
      if (!this.primitive) return;
      var cartographic = Cesium.Cartographic.fromCartesian(this.primitive._primitive._boundingSpheres[0].center);
      var surface = Cesium.Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, this.baseHeight);
      var offset = Cesium.Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, h);
      var translation = Cesium.Cartesian3.subtract(offset, surface, new Cesium.Cartesian3());
      this.primitive._primitive.modelMatrix = Cesium.Matrix4.fromTranslation(translation);
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.primitive) {
        this.viewer.scene.primitives.remove(this.primitive);
        this.primitive = null;
      }
    }
  }]);

  return LimitHeight;
}();

function getPostStageFragmentShader(viewShed, isTerrain) {
  var usesDepthTexture = viewShed._usesDepthTexture;
  var polygonOffsetSupported = viewShed._polygonOffsetSupported;
  var isPointLight = viewShed._isPointLight;
  var isSpotLight = viewShed._isSpotLight;
  var hasCascades = viewShed._numberOfCascades > 1;
  var debugCascadeColors = viewShed.debugCascadeColors;
  var softShadows = viewShed.softShadows;
  var fsSource = '';

  if (isPointLight) {
    fsSource += '#define USE_CUBE_MAP_SHADOW\n';
  } else if (usesDepthTexture) {
    fsSource += '#define USE_SHADOW_DEPTH_TEXTURE\n';
  }

  if (softShadows && !isPointLight) {
    fsSource += '#define USE_SOFT_SHADOWS\n';
  } // 定义阴影贴图参数


  var shadowParameters = "struct sg_shadowParameters{ \n        #ifdef USE_CUBE_MAP_SHADOW\n\n            vec3 texCoords;\n\n        #else\n\n            vec2 texCoords;\n\n        #endif\n\n            float depthBias;\n            float depth;\n            float nDotL;\n            vec2 texelStepSize;\n            float normalShadingSmooth;\n            float darkness;\n        };\n";
  var shadowVisibility = '#ifdef USE_CUBE_MAP_SHADOW\n' + // 获取当前纹理的的深度
  'float sg_sampleShadowMap(samplerCube shadowMap, vec3 d)\n' + '{\n' + '    return czm_unpackDepth(textureCube(shadowMap, d));\n' + '}\n' + // 比较当前深度和某坐标点深度
  'float sg_shadowDepthCompare(samplerCube shadowMap, vec3 uv, float depth)\n' + '{\n' + '    return step(depth, sg_sampleShadowMap(shadowMap, uv));\n' + '}\n' + 'float sg_shadowVisibility(samplerCube shadowMap, sg_shadowParameters shadowParameters)\n' + '{\n' + '    float depthBias = shadowParameters.depthBias;\n' + '    float depth = shadowParameters.depth;\n' + '    float nDotL = shadowParameters.nDotL;\n' + '    float normalShadingSmooth = shadowParameters.normalShadingSmooth;\n' + '    float darkness = shadowParameters.darkness;\n' + '    vec3 uvw = shadowParameters.texCoords;\n' + '\n' + '    depth -= depthBias;\n' + '    float visibility = sg_shadowDepthCompare(shadowMap, uvw, depth);\n' + '    return visibility;\n' + '}\n' + '#else\n' + 'float sg_sampleShadowMap(sampler2D shadowMap, vec2 uv)\n' + '{\n' + '#ifdef USE_SHADOW_DEPTH_TEXTURE\n' + '    return texture(shadowMap, uv).r;\n' + '#else\n' + '    return czm_unpackDepth(texture(shadowMap, uv));\n' + '#endif\n' + '}\n' + 'float sg_shadowDepthCompare(sampler2D shadowMap, vec2 uv, float depth)\n' + '{\n' + '    return step(depth, sg_sampleShadowMap(shadowMap, uv));\n' + '}\n' + 'float sg_shadowVisibility(sampler2D shadowMap, sg_shadowParameters shadowParameters)\n' + '{\n' + '    float depthBias = shadowParameters.depthBias;\n' + '    float depth = shadowParameters.depth;\n' + '    float nDotL = shadowParameters.nDotL;\n' + '    float normalShadingSmooth = shadowParameters.normalShadingSmooth;\n' + '    float darkness = shadowParameters.darkness;\n' + '    vec2 uv = shadowParameters.texCoords;\n' + '\n' + '    depth -= depthBias;\n' + '#ifdef USE_SOFT_SHADOWS\n' + '    vec2 texelStepSize = shadowParameters.texelStepSize;\n' + '    float radius = 1.0;\n' + '    float dx0 = -texelStepSize.x * radius;\n' + '    float dy0 = -texelStepSize.y * radius;\n' + '    float dx1 = texelStepSize.x * radius;\n' + '    float dy1 = texelStepSize.y * radius;\n' + '    float visibility = (\n' + '        sg_shadowDepthCompare(shadowMap, uv, depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(dx0, dy0), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(0.0, dy0), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(dx1, dy0), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(dx0, 0.0), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(dx1, 0.0), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(dx0, dy1), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(0.0, dy1), depth) +\n' + '        sg_shadowDepthCompare(shadowMap, uv + vec2(dx1, dy1), depth)\n' + '    ) * (1.0 / 9.0);\n' + '#else\n' + '    float visibility = sg_shadowDepthCompare(shadowMap, uv, depth);\n' + '#endif\n' + '\n' + '    return visibility;\n' + '}\n' + '#endif\n';
  var getPostionEC = 'vec4 getPositionEC(float depth) \n' + '{ \n' + '    vec2 xy = vec2((v_textureCoordinates.x * 2.0 - 1.0), (v_textureCoordinates.y * 2.0 - 1.0));\n' + '    float z = (depth - czm_viewportTransformation[3][2]) / czm_viewportTransformation[2][2];\n' + '    vec4 posInCamera = czm_inverseProjection * vec4(xy, z, 1.0);\n' + '    posInCamera = posInCamera / posInCamera.w;\n' + '    return posInCamera;\n' + '} \n';
  fsSource += 'uniform sampler2D colorTexture;\n' + 'uniform sampler2D depthTexture;\n';

  if (isPointLight) {
    fsSource += 'uniform samplerCube shadowMap_textureCube; \n';
  } else {
    fsSource += 'uniform sampler2D shadowMap_texture; \n';
  }

  fsSource += 'uniform mat4 shadowMap_matrix; \n' + 'uniform vec3 shadowMap_lightDirectionEC; \n' + 'uniform vec4 shadowMap_lightPositionEC; \n' + 'uniform vec4 shadowMap_normalOffsetScaleDistanceMaxDistanceAndDarkness; \n' + 'uniform vec4 shadowMap_texelSizeDepthBiasAndNormalShadingSmooth; \n' + 'uniform vec4 viewShed_frontColor; \n' + 'uniform vec4 viewShed_backColor; \n' + 'uniform float viewShed_Fov; \n' + 'uniform float viewShed_Far;\n' + '\n' + 'in vec2 v_textureCoordinates;\n' + '\n' + shadowParameters + shadowVisibility + getPostionEC + 'vec3 getNormalEC() \n' + '{ \n' + '    return vec3(1.0); \n' + '} \n' + '\n';
  fsSource += 'void main() \n' + '{ \n' + '    float depth = czm_readDepth(depthTexture, v_textureCoordinates);\n' + '    if(depth > 0.999999)\n' + '    {\n' + '        out_FragColor = texture(colorTexture, v_textureCoordinates);\n' + '        return;\n' + '    }\n' + '    vec4 positionEC = getPositionEC(depth); \n' + '    vec3 normalEC = getNormalEC(); \n' + '    float z = -positionEC.z; \n';
  fsSource += '    sg_shadowParameters shadowParameters; \n' + '    shadowParameters.texelStepSize = shadowMap_texelSizeDepthBiasAndNormalShadingSmooth.xy; \n' + '    shadowParameters.depthBias = shadowMap_texelSizeDepthBiasAndNormalShadingSmooth.z; \n' + '    shadowParameters.normalShadingSmooth = shadowMap_texelSizeDepthBiasAndNormalShadingSmooth.w; \n' + '    shadowParameters.darkness = shadowMap_normalOffsetScaleDistanceMaxDistanceAndDarkness.w; \n';

  if (isTerrain) {
    // Scale depth bias based on view distance to reduce z-fighting in distant terrain
    fsSource += '    shadowParameters.depthBias *= max(z * 0.01, 1.0); \n';
  } else if (!polygonOffsetSupported) {
    // If polygon offset isn't supported push the depth back based on view, however this
    // causes light leaking at further away views
    fsSource += '    shadowParameters.depthBias *= mix(1.0, 100.0, z * 0.0015); \n';
  }

  if (isPointLight) {
    fsSource += '    vec3 directionEC = positionEC.xyz - shadowMap_lightPositionEC.xyz; \n' + '    float distance = length(directionEC); \n' + '    directionEC = normalize(directionEC); \n' + '    float radius = shadowMap_lightPositionEC.w; \n' + '    // Stop early if the fragment is beyond the point light radius \n' + '    if (distance > radius) \n' + '    { \n' + '        out_FragColor = texture(colorTexture, v_textureCoordinates);\n' + '        return; \n' + '    } \n' + '    vec3 directionWC  = czm_inverseViewRotation * directionEC; \n' + '    shadowParameters.depth = distance / radius; \n' + '    shadowParameters.texCoords = directionWC; \n' + '    float visibility = sg_shadowVisibility(shadowMap_textureCube, shadowParameters); \n';
  } else if (isSpotLight) {
    fsSource += '    vec3 directionEC = positionEC.xyz - shadowMap_lightPositionEC.xyz; \n' + '    float distance = length(directionEC); \n' + '    if(distance > viewShed_Far)\n' + '    {\n' + '        out_FragColor = texture(colorTexture, v_textureCoordinates);\n' + '        return;\n' + '    }\n' + '    vec4 shadowPosition = shadowMap_matrix * positionEC; \n' + '    // Spot light uses a perspective projection, so perform the perspective divide \n' + '    shadowPosition /= shadowPosition.w; \n' + '    // Stop early if the fragment is not in the shadow bounds \n' + '    if (any(lessThan(shadowPosition.xyz, vec3(0.0))) || any(greaterThan(shadowPosition.xyz, vec3(1.0)))) \n' + '    { \n' + '        out_FragColor = texture(colorTexture, v_textureCoordinates);\n' + '        return; \n' + '    } \n' + '    shadowParameters.texCoords = shadowPosition.xy; \n' + '    shadowParameters.depth = shadowPosition.z; \n' + '    float visibility = sg_shadowVisibility(shadowMap_texture, shadowParameters); \n';
  } else if (hasCascades) {
    fsSource += '    float maxDepth = shadowMap_cascadeSplits[1].w; \n' + '    // Stop early if the eye depth exceeds the last cascade \n' + '    if (z > maxDepth) \n' + '    { \n' + '        out_FragColor = texture(colorTexture, v_textureCoordinates);\n' + '        return; \n' + '    } \n' + '    // Get the cascade based on the eye-space z \n' + '    vec4 weights = czm_cascadeWeights(z); \n' + '    // Transform position into the cascade \n' + '    vec4 shadowPosition = czm_cascadeMatrix(weights) * positionEC; \n' + '    // Get visibility \n' + '    shadowParameters.texCoords = shadowPosition.xy; \n' + '    shadowParameters.depth = shadowPosition.z; \n' + '    float visibility = sg_shadowVisibility(shadowMap_texture, shadowParameters); \n' + '    // Fade out shadows that are far away \n' + '    float shadowMapMaximumDistance = shadowMap_normalOffsetScaleDistanceMaxDistanceAndDarkness.z; \n' + '    float fade = max((z - shadowMapMaximumDistance * 0.8) / (shadowMapMaximumDistance * 0.2), 0.0); \n' + '    visibility = mix(visibility, 1.0, fade); \n';
  } else {
    fsSource += '    vec4 shadowPosition = shadowMap_matrix * positionEC; \n' + '    // Stop early if the fragment is not in the shadow bounds \n' + '    if (any(lessThan(shadowPosition.xyz, vec3(0.0))) || any(greaterThan(shadowPosition.xyz, vec3(1.0)))) \n' + '    { \n' + '        out_FragColor = texture(colorTexture, v_textureCoordinates);\n' + '        return; \n' + '    } \n' + '    shadowParameters.texCoords = shadowPosition.xy; \n' + '    shadowParameters.depth = shadowPosition.z; \n' + '    float visibility = sg_shadowVisibility(shadowMap_texture, shadowParameters); \n';
  }

  fsSource += '    vec4 color = texture(colorTexture, v_textureCoordinates);\n' + (hasCascades && debugCascadeColors ? '    color *= czm_cascadeColor(weights); \n' : '') + '    if(visibility > 0.0) \n' + '        out_FragColor = vec4(color.rgb * (1.0 - viewShed_frontColor.a) + viewShed_frontColor.rgb * viewShed_frontColor.a, color.a); \n' + '    else \n' + '        out_FragColor = vec4(color.rgb * (1.0 - viewShed_backColor.a) + viewShed_backColor.rgb * viewShed_backColor.a, color.a); \n' + '} \n';
  return fsSource;
}

var VisualField = /*#__PURE__*/function () {
  /**
   * 
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} options  基础配置
   * @param {Object} options.cameraOptions  视锥体参数
   * @param {Cesium.Cartesian3} options.cameraOptions.viewerPosition 视锥体顶点位置
   * @param {Number} [options.cameraOptions.heading=0]  视锥体偏转角
   * @param {Number} [options.cameraOptions.pitch=0]  视锥体仰俯角
   * @param {Number} [options.cameraOptions.horizontalFov=120]  水平张角
   * @param {Number} [options.cameraOptions.verticalFov=60]  垂直张角
   * @param {Number} [options.cameraOptions.distance=100]  视锥体长度
   * @param {String} options.cameraOptions.visibleAreaColor  可见区域颜色
   * @param {Number} [options.cameraOptions.visibleAreaColorAlpha=1]  可见区域颜色透明度
   * @param {String} options.cameraOptions.hiddenAreaColor  不可见区域颜色
   * @param {Number} [options.cameraOptions.hiddenAreaColorAlpha=1]  不可见区域颜色透明度
   * @param {Boolean} [options.enabled=120]  是否开启阴影贴图
   * @param {Number} [options.size=2048]  阴影贴图的大小
   * @param {Boolean} [options.softShadows=false]  
   * @param {Cesium.Color} [options.outlineColor=Cesium.Color.YELLOW]  锥体边框线颜色
   */
  function VisualField(viewer, options) {
    _classCallCheck(this, VisualField);

    if (!Cesium.defined(viewer)) {
      throw new Cesium.DeveloperError('缺少地图对象！');
    }

    this.options = options || {};
    this._scene = viewer.scene;
    var cameraOptions = options.cameraOptions || {};
    /**
     * @property {Boolean} _enabled 是否开启点阴影贴图
     */

    this._enabled = Cesium.defaultValue(options.enabled, true);
    /**
    * @property {Cesium.Cartesian3} _viewerPosition 视锥体顶点位置
    */

    this._viewerPosition = Cesium.defaultValue(cameraOptions.viewerPosition, new Cesium.Cartesian3.fromDegrees(0, 0, 0));
    /**
    * @property {Number} _heading 偏转角
    */

    this._heading = Cesium.defaultValue(cameraOptions.heading, 0);
    /**
     * @property {Number} _heading 仰俯角
     */

    this._pitch = Cesium.defaultValue(cameraOptions.pitch, 0);
    /**
    * @property {Number} _horizontalFov 水平视角范围
    */

    this._horizontalFov = Cesium.defaultValue(cameraOptions.horizontalFov, 120);
    /**
     * @property {Number} _verticalFov 垂直视角范围
     */

    this._verticalFov = Cesium.defaultValue(cameraOptions.verticalFov, 60);
    /**
     * @property {Number} _distance 视锥体长度
     */

    this._distance = Cesium.defaultValue(cameraOptions.distance, 100);
    /**
     * @property {Cesium.Color} _visibleAreaColor 可见区域颜色
     */

    this._visibleAreaColor = cameraOptions.visibleAreaColor instanceof Cesium.Color ? cameraOptions.visibleAreaColor : Cesium.Color.fromCssColorString(cameraOptions.visibleAreaColor); // 可见区域颜色透明度

    this._visibleAreaColorAlpha = cameraOptions.visibleAreaColorAlpha == undefined ? 1 : cameraOptions.visibleAreaColorAlpha;
    /**
     * @property {Cesium.Color} _hiddenAreaColor不可见区域颜色
     */

    this._hiddenAreaColor = cameraOptions.hiddenAreaColor instanceof Cesium.Color ? cameraOptions.hiddenAreaColor : Cesium.Color.fromCssColorString(cameraOptions.hiddenAreaColor); // 不可见地区颜色透明度

    this._hiddenAreaColorAlpha = cameraOptions.hiddenAreaColorAlpha == undefined ? 1 : cameraOptions.hiddenAreaColorAlpha; // 阴影贴图的像素大小尺寸

    this._size = Cesium.defaultValue(options.size, 2048); // 阴影贴图的柔和阴影

    this._softShadows = Cesium.defaultValue(options.softShadows, false); // 屏蔽距离误差

    this._bugDistance = this._distance + 0.000001 * this._horizontalFov - 0.000001 * this._verticalFov; // 椎体边界颜色

    this._outlineColor = Cesium.defaultValue(options.outlineColor, Cesium.Color.YELLOW); // 构建视锥体

    this._lightCameraPrimitive = undefined; // 构建光源相机

    this._lightCamera = new Cesium.Camera(this._scene); // 控制椎体相机改变

    this._lightCameraDirty = false; // 添加后处理

    this._stage = undefined;
    this._stageDirty = true;
    this.updateCamera(); // 创建一个点光源

    this._shadowMap = new Cesium.ShadowMap({
      context: this._scene.context,
      lightCamera: this._lightCamera,
      enabled: this._enabled,
      isPointLight: false,
      pointLightRadius: 100.0,
      cascadesEnabled: false,
      size: this._size,
      softShadows: this._softShadows,
      normalOffset: false,
      fromLightSource: false
    });
    this._bias = this._shadowMap._primitiveBias;
  }

  _createClass(VisualField, [{
    key: "enabled",
    get: function get() {
      return this._enabled;
    },
    set: function set(value) {
      /* this.dirty = this._enabled !== value; */
      this._enabled = value;
      this._shadowMap.enabled = value;
    }
  }, {
    key: "softShadows",
    get: function get() {
      return this._softShadows;
    },
    set: function set(value) {
      this._softShadows = value;
      this._shadowMap.softShadows = value;
    }
  }, {
    key: "size",
    get: function get() {
      return this._size;
    },
    set: function set(value) {
      this.size = value;
      this._shadowMap.size = value;
    }
  }, {
    key: "visibleAreaColor",
    get: function get() {
      return Cesium.Color.fromCartesian4(this._visibleAreaColor);
    },
    set: function set(value) {
      var color = value instanceof Cesium.Color ? value : Cesium.Color.fromCssColorString(value);
      this._visibleAreaColor = color;

      this._scene.requestRender();
    }
  }, {
    key: "visibleAreaColorAlpha",
    get: function get() {
      return this._visibleAreaColorAlpha;
    },
    set: function set(value) {
      this._visibleAreaColorAlpha = Number(value);

      this._scene.requestRender();
    }
  }, {
    key: "hiddenAreaColorAlpha",
    get: function get() {
      return this._hiddenAreaColorAlpha;
    },
    set: function set(value) {
      this._hiddenAreaColorAlpha = Number(value);

      this._scene.requestRender();
    }
  }, {
    key: "hiddenAreaColor",
    get: function get() {
      return Cesium.Color.fromCartesian4(this._hiddenAreaColor);
    },
    set: function set(value) {
      var color = value instanceof Cesium.Color ? value : Cesium.Color.fromCssColorString(value);
      this._hiddenAreaColor = color;
      /* this._hiddenAreaColor = Cesium.Cartesian4.fromColor(color); */

      this._scene.requestRender();
    }
  }, {
    key: "viewerPosition",
    get: function get() {
      return this._viewerPosition;
    },
    set: function set(value) {
      this._viewerPosition = value;
      this._lightCameraDirty = true;

      this._scene.requestRender();
    }
  }, {
    key: "heading",
    get: function get() {
      return this._heading;
    },
    set: function set(value) {
      this._heading = value;
      this._lightCameraDirty = true;

      this._scene.requestRender();
    }
  }, {
    key: "pitch",
    get: function get() {
      return this._pitch;
    },
    set: function set(value) {
      this._pitch = value;
      this._lightCameraDirty = true;

      this._scene.requestRender();
    }
  }, {
    key: "horizontalFov",
    get: function get() {
      return this._horizontalFov;
    },
    set: function set(value) {
      this._horizontalFov = value;
      this._bugDistance = this._distance + 0.000001 * this._horizontalFov - 0.000001 * this._verticalFov;
      this._lightCameraDirty = true;

      this._scene.requestRender();
    }
  }, {
    key: "verticalFov",
    get: function get() {
      return this._verticalFov;
    },
    set: function set(value) {
      this._verticalFov = value;
      this._bugDistance = this._distance + 0.000001 * this._horizontalFov - 0.000001 * this._verticalFov;
      this._lightCameraDirty = true;

      this._scene.requestRender();
    }
  }, {
    key: "distance",
    get: function get() {
      return this._distance;
    },
    set: function set(value) {
      this._distance = value;
      this._bugDistance = this._distance + 0.000001 * this._horizontalFov - 0.000001 * this._verticalFov;
      this._lightCameraDirty = true;

      this._scene.requestRender();
    } // 锥体相机更新

  }, {
    key: "updateCamera",
    value: function updateCamera() {
      // 视锥体近平面
      this._lightCamera.frustum.near = .001 * this._bugDistance; // 视锥体远平面

      this._lightCamera.frustum.far = this._bugDistance; // 视锥体张角

      this._lightCamera.frustum.fov = Cesium.Math.toRadians(this._verticalFov); // 视锥体宽高比

      var horizontalFovRadians = Cesium.Math.toRadians(this._horizontalFov);
      var verticalFovRadians = Cesium.Math.toRadians(this._verticalFov);
      /*  this._lightCamera.frustum.aspectRatio = (this._bugDistance * Math.tan(horizontalFovRadians * 0.5) * 2.0) / (this._bugDistance * Math.tan(verticalFovRadians * 0.5) * 2.0); */
      // 如果水平方向张角大于垂直方向 则视锥体张角取值为水平方向角度 ？

      /*  if (this._horizontalFov > this._verticalFov) this._lightCamera.frustum.fov = Cesium.Math.toRadians(this._horizontalFov); */

      this._lightCamera.frustum.aspectRatio = this._bugDistance * Math.tan(horizontalFovRadians * 0.5) * 2.0 / (this._bugDistance * Math.tan(verticalFovRadians * 0.5) * 2.0); //定义视锥体的视场角度为水平方向角度

      if (this._horizontalFov > this._verticalFov) this._lightCamera.frustum.fov = Cesium.Math.toRadians(this._horizontalFov); // 设置相机姿态

      this._lightCamera.setView({
        destination: this._viewerPosition,
        orientation: {
          heading: Cesium.Math.toRadians(this._heading),
          pitch: Cesium.Math.toRadians(this._pitch)
        }
      }); // 构建视锥体


      if (this._lightCameraPrimitive) {
        this._lightCameraPrimitive.destroy();

        this._lightCameraPrimitive = undefined;
      }

      var outlineGeometry = this.createOutLineGeometry();
      this._lightCameraPrimitive = new Cesium.Primitive({
        geometryInstances: new Cesium.GeometryInstance({
          geometry: outlineGeometry,
          attributes: {
            color: Cesium.ColorGeometryInstanceAttribute.fromColor(this._outlineColor)
          }
        }),
        appearance: new Cesium.PerInstanceColorAppearance({
          translucent: false,
          flat: true
        }),
        modelMatrix: this._lightCamera.inverseViewMatrix,
        asynchronous: false
      });
      this._lightCameraDirty = false;
    } // 构建谁锥体几何

  }, {
    key: "createOutLineGeometry",
    value: function createOutLineGeometry() {
      var positions = new Float32Array(633);
      var i,
          a,
          s,
          d,
          p = positions,
          m = Cesium.Math.toRadians(this._horizontalFov),
          v = Cesium.Math.toRadians(this._verticalFov),
          b = Math.tan(0.5 * m),
          S = Math.tan(0.5 * v);
      a = this._distance * b;
      d = this._distance * S;
      i = -a;
      s = -d;
      var P = 0;
      p[P++] = 0;
      p[P++] = 0;
      p[P++] = 0;
      var D, I;
      var M = Math.PI - 0.5 * m;
      var R = m / 4;

      for (var L = 0; L < 5; ++L) {
        D = M + L * R;
        var B = d / (this._distance / Math.cos(D));
        var F = Math.atan(B);
        var U = -F;
        var V = F / 10;

        for (var z = 0; z < 21; ++z) {
          I = U + z * V;
          p[P++] = this._distance * Math.cos(I) * Math.sin(D);
          p[P++] = this._distance * Math.sin(I);
          p[P++] = this._distance * Math.cos(I) * Math.cos(D);
        }
      }

      R = m / 20;

      for (var G = 0; G < 21; ++G) {
        D = M + G * R;

        var _B = d / (this._distance / Math.cos(D));

        var _F = Math.atan(_B);

        var _U = -_F,
            _V = _F / 2;

        for (var H = 0; H < 5; ++H) {
          I = _U + H * _V;
          p[P++] = this._distance * Math.cos(I) * Math.sin(D);
          p[P++] = this._distance * Math.sin(I);
          p[P++] = this._distance * Math.cos(I) * Math.cos(D);
        }
      }

      var attributes = new Cesium.GeometryAttributes({
        position: new Cesium.GeometryAttribute({
          componentDatatype: Cesium.ComponentDatatype.DOUBLE,
          componentsPerAttribute: 3,
          values: positions
        })
      });
      var indices = new Uint16Array(408);
      var t = indices;
      var r = 0;
      t[r++] = 0;
      t[r++] = 1;
      t[r++] = 0;
      t[r++] = 21;
      t[r++] = 0;
      t[r++] = 85;
      t[r++] = 0;
      t[r++] = 105;

      for (var i = 0, n = 0; n < 5; ++n) {
        i++;

        for (var _a = 0; _a < 20; ++_a) {
          t[r++] = i++, t[r++] = i;
        }
      }

      i++;

      for (var s = 0; s < 20; ++s) {
        for (var l = 0; l < 5; ++l) {
          t[r++] = i, t[r++] = i++ + 5;
        }
      }

      return new Cesium.Geometry({
        attributes: attributes,
        indices: indices,
        primitiveType: Cesium.PrimitiveType.LINES,
        boundingSphere: Cesium.BoundingSphere.fromVertices(positions)
      });
      /* let positions = new Float64Array(5 * 3);
      // 点0 坐标
      positions[0] = 0.5;
      positions[1] = 0.0;
      positions[2] = 0.5;
        // 点1 坐标
      positions[3] = 0.0;
      positions[4] = -1.0;
      positions[5] = 0.0;
        // 点2 坐标
      positions[6] = 1.0;
      positions[7] = -1.0;
      positions[8] = 0.0;
        // 点3 坐标
      positions[9] = 1.0;
      positions[10] = -1.0;
      positions[11] = 1.0;
        // 点4 坐标
      positions[12] = 0.0;
      positions[13] = -1.0;
      positions[14] = 1.0;
        let that = this;
      let arr = [];
      positions.forEach(function (item) {
          let val = item * that._distance;
          arr.push(val);
      });
      positions = arr;
        // 创建顶点属性中的坐标
      const attributes = new Cesium.GeometryAttributes({
          position: new Cesium.GeometryAttribute({
              componentDatatype: Cesium.ComponentDatatype.DOUBLE,
              componentsPerAttribute: 3,
              values: positions
          })
      });
        // 点的索引
      const indices = new Uint16Array(18);
        indices[0] = 0;
      indices[1] = 4;
        indices[2] = 0;
      indices[3] = 1;
        indices[4] = 0;
      indices[5] = 2;
        indices[6] = 0;
      indices[7] = 3;
        indices[8] = 1;
      indices[9] = 4;
        indices[10] = 4;
      indices[11] = 1;
        indices[12] = 1;
      indices[13] = 2;
        indices[14] = 2;
      indices[15] = 3;
        indices[16] = 3;
      indices[17] = 4;
        let geometry = new Cesium.Geometry({
          attributes: attributes,
          indices: indices,
          primitiveType: Cesium.PrimitiveType.LINES,
          boundingSphere: Cesium.BoundingSphere.fromVertices(positions)
      }); 
        return geometry;*/
    } // 更新后处理

  }, {
    key: "updateStage",
    value: function updateStage() {
      if (!this._stageDirty) {
        return;
      }

      this._stageDirty = false;

      if (Cesium.defined(this._stage)) {
        this._scene.postProcessStages.remove(this._stage);

        this._stage = undefined;
      }

      var scratchTexelStepSize = new Cesium.Cartesian2();
      var bias = this._bias;
      var shadowMap = this._shadowMap;
      var that = this;
      var uniformMap = {
        shadowMap_texture: function shadowMap_texture() {
          return shadowMap._shadowMapTexture;
        },
        shadowMap_matrix: function shadowMap_matrix() {
          return shadowMap._shadowMapMatrix;
        },
        viewShed_frontColor: function viewShed_frontColor() {
          var vColor = that._visibleAreaColor.withAlpha(that._visibleAreaColorAlpha);

          vColor = Cesium.Cartesian4.fromColor(vColor);
          return vColor;
        },
        viewShed_backColor: function viewShed_backColor() {
          var hColor = that._hiddenAreaColor.withAlpha(that._hiddenAreaColorAlpha);

          hColor = Cesium.Cartesian4.fromColor(hColor);
          return hColor;
        },
        viewShed_Far: function viewShed_Far() {
          return shadowMap._lightCamera.frustum.far;
        },
        shadowMap_lightheadingEC: function shadowMap_lightheadingEC() {
          return shadowMap._lightheadingEC;
        },
        shadowMap_lightPositionEC: function shadowMap_lightPositionEC() {
          return shadowMap._lightPositionEC;
        },
        shadowMap_texelSizeDepthBiasAndNormalShadingSmooth: function shadowMap_texelSizeDepthBiasAndNormalShadingSmooth() {
          var texelStepSize = scratchTexelStepSize;
          texelStepSize.x = 1.0 / shadowMap._textureSize.x;
          texelStepSize.y = 1.0 / shadowMap._textureSize.y;
          return Cesium.Cartesian4.fromElements(texelStepSize.x, texelStepSize.y, bias.depthBias, bias.normalShadingSmooth, this.combinedUniforms1);
        },
        shadowMap_normalOffsetScaleDistanceMaxDistanceAndDarkness: function shadowMap_normalOffsetScaleDistanceMaxDistanceAndDarkness() {
          return Cesium.Cartesian4.fromElements(bias.normalOffsetScale, shadowMap._distance, shadowMap.maximumDistance, shadowMap._darkness, this.combinedUniforms2);
        },
        combinedUniforms1: new Cesium.Cartesian4(),
        combinedUniforms2: new Cesium.Cartesian4()
      };
      var fshader = getPostStageFragmentShader(shadowMap, false);
      this._stage = new Cesium.PostProcessStage({
        fragmentShader: fshader,
        uniforms: uniformMap
      });

      this._scene.postProcessStages.add(this._stage);
    }
  }, {
    key: "update",
    value: function update(frameState) {
      if (this._lightCameraDirty) this.updateCamera();
      this.updateStage();
      frameState.shadowMaps.push(this._shadowMap);
      if (this._lightCameraPrimitive) this._lightCameraPrimitive.update(frameState);
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (Cesium.defined(this._stage)) {
        this._scene.postProcessStages.remove(this._stage);

        this._stage = undefined;
        /*  var length = this._scene.postProcessStages.length;  */
      }

      this._shadowMap = this._shadowMap.destroy();

      if (this._lightCameraPrimitive) {
        this._lightCameraPrimitive.destroy();

        this._lightCameraPrimitive = undefined;
      }
    }
  }]);

  return VisualField;
}();

/**
 * 可视域控制类
 * @description 可视域控制类，通过此类对象，可直接添加可视域，并对添加的可视域进行控制，而不用单独创建可视域。
 * @class 
 */

var visualFieldTool = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象
   
   * 
   */
  function visualFieldTool(viewer, opt) {
    _classCallCheck(this, visualFieldTool);

    if (!Cesium.defined(viewer)) {
      throw new Cesium.DeveloperError('缺少地图对象！');
    }

    this.viewer = viewer;
    this.handler = undefined;
    this.prompt = null;
    this.vfPrimitive = null;
    /**
     * @property {Array} vfPrimitiveArr 可视域对象数组
     */

    this.vfPrimitiveArr = [];
  }
  /** 
  * 事件绑定
  * @param {String} type 事件类型（startEdit 开始编辑时 / endEdit 编辑结束时 / remove 删除对象时 / endCreate 创建完成后）
  * @param {Function} fun 绑定函数
  */


  _createClass(visualFieldTool, [{
    key: "on",
    value: function on(type, fun) {
      if (type == "startEdit") {
        // 开始编辑事件
        this.startEditFun = fun;
      } else if (type == "endEdit") {
        // 结束编辑事件
        this.endEditFun = fun;
      } else if (type == "remove") {
        // 移除事件
        this.removeFun = fun;
      } else if (type == "endCreate") {
        // 绘制完成事件
        this.endCreateFun = fun;
      }
    }
    /**
     * 绘制可视域
     * @param {Object} opt 当前可视域的配置 
     * @param {String} [opt.visibleAreaColorAlpha="#00FF00"] 可见区域颜色
     * @param {Number} [opt.visibleAreaColorAlpha=1] 可见区域颜色透明度
     * @param {String} [opt.hiddenAreaColorAlpha="#FF0000"] 不可见区域颜色
     * @param {Number} [opt.hiddenAreaColorAlpha=1] 不可见区域颜色透明度
     * @param {Number} [opt.verticalFov=60] 视锥体水平张角
    *  @param {Number} [opt.horizontalFov=120] 视锥体垂直张角
     * @param {Function} fun 绘制成功后的回调函数fun(vfPrimitive)
     */

  }, {
    key: "startDraw",
    value: function startDraw(opt, fun) {
      var that = this; // 默认样式

      var defaultStyle = {
        visibleAreaColor: "#00FF00",
        visibleAreaColorAlpha: 1,
        hiddenAreaColor: "#FF0000",
        hiddenAreaColorAlpha: 1,
        verticalFov: 60,
        horizontalFov: 120
      };
      opt = Object.assign(defaultStyle, opt || {});
      opt.id = opt.id || Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0)); // 给个默认id

      var visibleAreaColor = opt.visibleAreaColor;
      var hiddenAreaColor = opt.hiddenAreaColor;
      var visibleAreaColorAlpha = opt.visibleAreaColorAlpha;
      var hiddenAreaColorAlpha = opt.hiddenAreaColorAlpha;
      var verticalFov = opt.verticalFov;
      var horizontalFov = opt.horizontalFov;
      var startPosition = undefined;
      var endPosition = undefined;
      var vfPrimitive = undefined; // 当前绘制的视锥体

      if (!this.prompt) this.prompt = new Prompt$1(this.viewer, this.promptStyle);
      if (!this.handler) this.handler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.handler.setInputAction(function (evt) {
        // 单击开始绘制
        var cartesian = that.getCatesian3FromPX(evt.position, that.viewer);
        if (!cartesian) return;

        if (!startPosition) {
          startPosition = cartesian.clone();
        } else {
          endPosition = cartesian.clone();

          if (that.handler) {
            that.handler.destroy();
            that.handler = null;
          }

          if (that.prompt) {
            that.prompt.destroy();
            that.prompt = null;
          }

          var c1 = Cesium.Cartographic.fromCartesian(startPosition.clone());
          var c2 = Cesium.Cartographic.fromCartesian(endPosition.clone());
          var angle = that.computeAngle(c1, c2);
          vfPrimitive.heading = angle;
          vfPrimitive.attr.heading = angle;
          var distance = Cesium.Cartesian3.distance(startPosition.clone(), endPosition.clone());
          vfPrimitive.distance = distance;
          vfPrimitive.attr.distance = distance; // 绘制完成后 置为空

          startPosition = undefined;
          endPosition = undefined;
          if (fun) fun(vfPrimitive);
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
      this.handler.setInputAction(function (evt) {
        // 移动时绘制线
        if (!startPosition) {
          that.prompt.update(evt.endPosition, "单击开始绘制");
          return;
        }

        that.prompt.update(evt.endPosition, "再次单击结束");
        var cartesian = that.getCatesian3FromPX(evt.endPosition, that.viewer);
        if (!cartesian) return;

        if (!vfPrimitive) {
          vfPrimitive = new VisualField(that.viewer, {
            cameraOptions: {
              viewerPosition: startPosition.clone(),
              visibleAreaColor: visibleAreaColor,
              visibleAreaColorAlpha: visibleAreaColorAlpha,
              hiddenAreaColor: hiddenAreaColor,
              hiddenAreaColorAlpha: hiddenAreaColorAlpha,
              horizontalFov: horizontalFov,
              verticalFov: verticalFov
            }
          });
          that.viewer.scene.primitives.add(vfPrimitive);
          that.vfPrimitiveArr.push(vfPrimitive);
          vfPrimitive.attr = opt; // 属性绑定
        }

        var c1 = Cesium.Cartographic.fromCartesian(startPosition.clone());
        var c2 = Cesium.Cartographic.fromCartesian(cartesian.clone());
        var angle = that.computeAngle(c1, c2);
        vfPrimitive.heading = angle;
        vfPrimitive.attr.heading = angle;
        var distance = Cesium.Cartesian3.distance(startPosition.clone(), cartesian.clone());
        vfPrimitive.distance = distance;
        vfPrimitive.attr.distance = distance;
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
    /**
     * 设置可视区域颜色 
     * @param {Object} vfPrimitive 可视域对象 
     * @param {String} val 可视区域颜色
     */

  }, {
    key: "setVisibleAreaColor",
    value: function setVisibleAreaColor(vfPrimitive, val) {
      if (!val) return;
      this.visibleAreaColor = val;
      if (vfPrimitive) vfPrimitive.visibleAreaColor = val;
    }
    /**
     * 设置可视区域颜色透明度
     * @param {Object} vfPrimitive 可视域对象 
     * @param {Number} val 可视区域颜色透明度
     */

  }, {
    key: "setVisibleAreaColorAlpha",
    value: function setVisibleAreaColorAlpha(vfPrimitive, val) {
      if (!val) return;
      this.visibleAreaColorAlpha = Number(val);
      if (vfPrimitive) vfPrimitive.visibleAreaColorAlpha = Number(val);
    }
    /**
     * 设置不可视区域颜色
     * @param {Object} vfPrimitive 可视域对象 
     * @param {String} val 不可视区域颜色
     */

  }, {
    key: "setHiddenAreaColor",
    value: function setHiddenAreaColor(vfPrimitive, val) {
      if (!val) return;
      this.hiddenAreaColor = val;
      if (vfPrimitive) vfPrimitive.hiddenAreaColor = val;
    }
    /**
     * 设置不可视区域颜色透明度
     * @param {Object} vfPrimitive 可视域对象 
    * @param {Number} val 不可视区域颜色透明度
    */

  }, {
    key: "setHiddenAreaColorAlpha",
    value: function setHiddenAreaColorAlpha(vfPrimitive, val) {
      if (!val) return;
      this.hiddenAreaColorAlpha = Number(val);
      if (vfPrimitive) vfPrimitive.hiddenAreaColorAlpha = Number(val);
    }
    /**
     * 设置设置锥体长度
     * @param {Object} vfPrimitive 可视域对象 
    * @param {Number} val 锥体长度
    */

  }, {
    key: "setDistance",
    value: function setDistance(vfPrimitive, val) {
      if (!val) return;
      this.distance = Number(val);
      if (vfPrimitive) vfPrimitive.distance = Number(val);
    }
    /**
     * 设置垂直张角
     * @param {Object} vfPrimitive 可视域对象 
    * @param {Number} val 垂直张角
    */

  }, {
    key: "setVerticalFov",
    value: function setVerticalFov(vfPrimitive, val) {
      if (!val) return;
      this.verticalFov = Number(val);
      if (vfPrimitive) vfPrimitive.verticalFov = Number(val);
    }
    /**
     * 设置水平张角
     * @param {Object} vfPrimitive 可视域对象 
    * @param {Number} val 水平张角
    */

  }, {
    key: "setHorizontalFov",
    value: function setHorizontalFov(vfPrimitive, val) {
      if (!val) return;
      var value = Number(val);
      value = value >= 180 ? 179 : value; // 水平张角不超过180

      this.horizontalFov = Number(value);
      if (vfPrimitive) vfPrimitive.horizontalFov = Number(value);
    }
    /**
    * 设置偏转角
    * @param {Object} vfPrimitive 可视域对象 
    * @param {Number} val 偏转角
    */

  }, {
    key: "setHeading",
    value: function setHeading(vfPrimitive, val) {
      if (!val) return;
      this.heading = 0;
      if (vfPrimitive) vfPrimitive.heading = Number(val);
    }
    /**
    * 设置仰俯角
    * @param {Object} vfPrimitive 可视域对象 
    * @param {Number} val 仰俯角
    */

  }, {
    key: "setPitch",
    value: function setPitch(vfPrimitive, val) {
      if (!val) return;
      this.pitch = Number(val);
      if (vfPrimitive) vfPrimitive.pitch = Number(val);
    } // 计算两点朝向

  }, {
    key: "computeAngle",
    value: function computeAngle(p1, p2) {
      if (!p1 || !p2) return;
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

      bearing = bearing % 360;
      return bearing;
    } // 坐标拾取

  }, {
    key: "getCatesian3FromPX",
    value: function getCatesian3FromPX(px) {
      var picks = this.viewer.scene.drillPick(px);
      this.viewer.scene.render();
      var cartesian;
      var isOn3dtiles = false;

      for (var i = 0; i < picks.length; i++) {
        if (picks[i] && picks[i].primitive && picks[i].primitive instanceof Cesium.Cesium3DTileset) {
          //模型上拾取
          isOn3dtiles = true;
          break;
        }
      }

      if (isOn3dtiles) {
        cartesian = this.viewer.scene.pickPosition(px);
      } else {
        var ray = this.viewer.camera.getPickRay(px);
        if (!ray) return null;
        cartesian = this.viewer.scene.globe.pick(ray, this.viewer.scene);
        //sin add 20240718
        if (!cartesian) {
          cartesian = this.viewer.camera.pickEllipsoid(
              px,
              this.viewer.scene.globe.ellipsoid
          );
        }
      }

      return cartesian;
    }
    /**
    * 清除可视域
    */

  }, {
    key: "clear",
    value: function clear() {
      for (var i = 0; i < this.vfPrimitiveArr.length; i++) {
        var vfPrimitive = this.vfPrimitiveArr[i];
        this.viewer.scene.primitives.remove(vfPrimitive);
        vfPrimitive = null;
      }

      this.vfPrimitiveArr = [];
    }
    /**
    * 清除可视域 同clear，方法兼容
    */

  }, {
    key: "removeAll",
    value: function removeAll() {
      for (var i = 0; i < this.vfPrimitiveArr.length; i++) {
        var vfPrimitive = this.vfPrimitiveArr[i];
        this.viewer.scene.primitives.remove(vfPrimitive);
        vfPrimitive = null;
      }

      this.vfPrimitiveArr = [];
    }
    /**
     * 销毁
    */

  }, {
    key: "destroy",
    value: function destroy() {
      this.clear();

      if (this.handler) {
        this.handler.destroy();
        this.handler = null;
      }

      if (this.prompt) {
        this.prompt.destroy();
        this.prompt = null;
      }
    }
    /**
     * 根据startDraw中传入的字段属性来获取对应vfPrimitives
     * @param {String} [fieldName='id'] 字段名 
     * @param {String} fieldVlue 字段值
     * @returns {Array} vfprimitives，可视域对象数组
     */

  }, {
    key: "getVfPrimitiveByField",
    value: function getVfPrimitiveByField(fieldName, fieldVlue) {
      fieldName = fieldName || 'id';
      var vfprimitives = this.vfPrimitiveArr.filter(function (item) {
        return item.attr[fieldName] = fieldVlue;
      });
      return vfprimitives;
    }
    /**
     * 根据id属性来获取对应vfPrimitive
     * @param {String} id 唯一id 
     * @returns {Object} vpObj（vpObj.vfPrimitive 可视域 / vpObj.index 在数组中位置）
     */

  }, {
    key: "getVfPrimitiveById",
    value: function getVfPrimitiveById(id) {
      var vpObj = {};

      for (var i = 0; i < this.vfPrimitiveArr.length; i++) {
        var vp = this.vfPrimitiveArr[i];

        if (vp.attr.id == id) {
          vpObj.vfPrimitive = vp;
          vpObj.index = i;
          break;
        }
      }

      return vpObj;
    }
    /**
     * 根据id属性删除vfPrimitive
     * @param {String} id 唯一id 
     * @returns {Object} vpObj vpObj.vfPrimitive 可视域 / vpObj.index 在数组中位置
     */

  }, {
    key: "removeVfPrimitiveById",
    value: function removeVfPrimitiveById(id) {
      if (!id) return;
      var vpObj = this.getVfPrimitiveById(id);
      if (vpObj.vfPrimitive) this.viewer.scene.primitives.remove(vpObj.vfPrimitive);
      this.vfPrimitiveArr.splice(vpObj.index, 1);
    }
    /**
     * 删除单个可视域
     * @param {Object} vfPrimitive 可视域对象
     */

  }, {
    key: "removeOne",
    value: function removeOne(vfPrimitive) {
      if (!vfPrimitive) return;
      this.removeVfPrimitiveById(vfPrimitive.attr.id);
    }
  }]);

  return visualFieldTool;
}();

/**
 * @class
 * @description 高德poi查询，参考文档：https://lbs.amap.com/api/webservice/guide/api/search
 */

var GaodePOI = /*#__PURE__*/function () {
  /**
   * 
   * @param {Object} opt 
   * @param {Array | String} opt.keys 高德key，可不传key、传key数组以及key字符串
   */
  function GaodePOI(opt) {
    _classCallCheck(this, GaodePOI);

    this.opt = opt || {};
    var defaultKeys = ['a73e387f642573295b498d7fd6b4c537'];
    this._key = undefined; // 支持不传key、传key数组以及key字符串

    if (!this.opt.keys || this.opt.keys.length == 0) {
      this._key = defaultKeys[Math.floor(Math.random() * defaultKeys.length)];
    } else {
      if (this.opt.keys instanceof Array) {
        this._key = this.opt.keys[Math.floor(Math.random() * this.opt.keys.length)];
      } else {
        this._key = this.opt.keys;
      }
    }
  }

  _createClass(GaodePOI, [{
    key: "key",
    get: function get() {
      return this._key;
    },
    set: function set(key) {
      this._key = key;
    }
    /**
     * 关键字搜索
     * @param {Object} options  参数，可参考高德官网配置
     * @param {Function} success 成功后的回调函数 
     */

  }, {
    key: "queryText",
    value: function queryText(options, success) {
      var url = "https://restapi.amap.com/v3/place/text";
      var params = {
        key: this._key,
        offset: options.pageSize || 25,
        // 每页条数
        page: options.pageNumber || 1 // 当前页数

      };
      params = Object.assign(params, options || {});
      var that = this;
      axios.get(url, {
        params: params
      }).then(function (res) {
        var pois = res.data.pois || [];

        for (var i = 0; i < pois.length; i++) {
          var poi = pois[i];
          var location = poi.location;
          location = location.split(",");
          var coor = that.gcj2wgs(location);
          poi.lnglat = coor;
        }

        if (success) success(res.data.pois);
      });
    }
    /**
     * 周边搜索
     * @param {Object} options  参数，可参考高德官网配置
     * @param {Array} options.center  中心点经纬度坐标
     * @param {Function} success 成功后的回调函数 
     */

  }, {
    key: "queryAround",
    value: function queryAround(options, success) {
      if (!options || !options.center) {
        alert("缺少搜索中心点！");
        return;
      }

      var location = this.wgs2gcj(options.center);
      var url = "https://restapi.amap.com/v3/place/around";
      var params = {
        key: this._key,
        location: location[0] + "," + location[1],
        offset: options.pageSize || 25,
        // 每页条数
        page: options.pageNumber || 1,
        // 当前页数
        radius: options.radius // 默认半径 单位 米

      };
      var that = this;
      axios.get(url, {
        params: params
      }).then(function (res) {
        var pois = res.data.pois || [];

        for (var i = 0; i < pois.length; i++) {
          var poi = pois[i];
          var _location = poi.location;
          _location = _location.split(",");
          var coor = that.gcj2wgs(_location);
          poi.lnglat = coor;
        }

        if (success) success(res.data.pois);
      });
    }
    /**
     * 范围搜索
     * @param {Object} options  参数，可参考高德官网配置
     *  @param {Object} options.lnglats  范围面经纬度坐标数组
     * @param {Function} success 成功后的回调函数 
     */

  }, {
    key: "queryPolygon",
    value: function queryPolygon(options, success) {
      if (!options || !options.lnglats || options.lnglats.length < 3) {
        alert("缺少搜索范围！");
        return;
      }

      var polygon = '';

      for (var i = 0; i < options.lnglats.length; i++) {
        var lnglat = options.lnglats[i];
        lnglat = this.wgs2gcj(lnglat);
        polygon += lnglat[0] + ',' + lnglat[1] + "|";
      }

      var firstlnglat = this.wgs2gcj(options.lnglats[0]);
      polygon += firstlnglat[0] + "," + firstlnglat[1];
      var url = "https://restapi.amap.com/v3/place/polygon";
      var params = {
        key: this._key,
        polygon: polygon,
        offset: options.pageSize || 25,
        // 每页条数
        page: options.pageNumber || 1 // 当前页数

      };
      var that = this;
      axios.get(url, {
        params: params
      }).then(function (res) {
        var pois = res.data.pois || [];

        for (var _i = 0; _i < pois.length; _i++) {
          var poi = pois[_i];
          var location = poi.location;
          location = location.split(",");
          var coor = that.gcj2wgs(location);
          poi.lnglat = coor;
        }

        if (success) success(res.data.pois);
      });
    } // 开始搜索

    /**
     * @param {Number} type 搜索类型（1、关键字搜索；2、周边搜索；3、范围搜索）
     * @param {Object} params 搜索参数
     */

  }, {
    key: "query",
    value: function query(type, params, success) {
      type = Number(type || 1);

      if (type == 1) {
        this.queryText(params, success);
      } else if (type == 2) {
        this.queryAround(params, success);
      } else if (type == 3) {
        this.queryPolygon(params, success);
      } else {
        alert("poi查询类型有误");
        return;
      }
    }
  }, {
    key: "transformWD",
    value: function transformWD(lng, lat) {
      var PI = 3.1415926535897932384626;
      var ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * Math.sqrt(Math.abs(lng));
      ret += (20.0 * Math.sin(6.0 * lng * PI) + 20.0 * Math.sin(2.0 * lng * PI)) * 2.0 / 3.0;
      ret += (20.0 * Math.sin(lat * PI) + 40.0 * Math.sin(lat / 3.0 * PI)) * 2.0 / 3.0;
      ret += (160.0 * Math.sin(lat / 12.0 * PI) + 320 * Math.sin(lat * PI / 30.0)) * 2.0 / 3.0;
      return ret;
    }
  }, {
    key: "transformJD",
    value: function transformJD(lng, lat) {
      var PI = 3.1415926535897932384626;
      var ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * Math.sqrt(Math.abs(lng));
      ret += (20.0 * Math.sin(6.0 * lng * PI) + 20.0 * Math.sin(2.0 * lng * PI)) * 2.0 / 3.0;
      ret += (20.0 * Math.sin(lng * PI) + 40.0 * Math.sin(lng / 3.0 * PI)) * 2.0 / 3.0;
      ret += (150.0 * Math.sin(lng / 12.0 * PI) + 300.0 * Math.sin(lng / 30.0 * PI)) * 2.0 / 3.0;
      return ret;
    }
  }, {
    key: "wgs2gcj",
    value: function wgs2gcj(arrdata) {
      var a = 6378245.0;
      var ee = 0.00669342162296594323;
      var PI = 3.1415926535897932384626;
      var lng = Number(arrdata[0]);
      var lat = Number(arrdata[1]);
      var dlat = this.transformWD(lng - 105.0, lat - 35.0);
      var dlng = this.transformJD(lng - 105.0, lat - 35.0);
      var radlat = lat / 180.0 * PI;
      var magic = Math.sin(radlat);
      magic = 1 - ee * magic * magic;
      var sqrtmagic = Math.sqrt(magic);
      dlat = dlat * 180.0 / (a * (1 - ee) / (magic * sqrtmagic) * PI);
      dlng = dlng * 180.0 / (a / sqrtmagic * Math.cos(radlat) * PI);
      var mglat = lat + dlat;
      var mglng = lng + dlng;
      mglng = Number(mglng.toFixed(6));
      mglat = Number(mglat.toFixed(6));
      return [mglng, mglat];
    }
  }, {
    key: "gcj2wgs",
    value: function gcj2wgs(arrdata) {
      var a = 6378245.0;
      var ee = 0.00669342162296594323;
      var PI = 3.1415926535897932384626;
      var lng = Number(arrdata[0]);
      var lat = Number(arrdata[1]);
      var dlat = this.transformWD(lng - 105.0, lat - 35.0);
      var dlng = this.transformJD(lng - 105.0, lat - 35.0);
      var radlat = lat / 180.0 * PI;
      var magic = Math.sin(radlat);
      magic = 1 - ee * magic * magic;
      var sqrtmagic = Math.sqrt(magic);
      dlat = dlat * 180.0 / (a * (1 - ee) / (magic * sqrtmagic) * PI);
      dlng = dlng * 180.0 / (a / sqrtmagic * Math.cos(radlat) * PI);
      var mglat = lat + dlat;
      var mglng = lng + dlng;
      var jd = lng * 2 - mglng;
      var wd = lat * 2 - mglat;
      jd = Number(jd.toFixed(6));
      wd = Number(wd.toFixed(6));
      return [jd, wd];
    }
  }]);

  return GaodePOI;
}();

var GaodeRoute = /*#__PURE__*/function () {
  /**
   * @param {Object} opt 
   * @param {Array | String} opt.keys 高德key，可不传key、传key数组以及key字符串
   */
  function GaodeRoute(opt) {
    _classCallCheck(this, GaodeRoute);

    this.opt = opt || {};
    var defaultKeys = ['a73e387f642573295b498d7fd6b4c537'];
    this._key = undefined; // 支持不传key、传key数组以及key字符串

    if (!this.opt.keys || this.opt.keys.length == 0) {
      this._key = defaultKeys[Math.floor(Math.random() * defaultKeys.length)];
    } else {
      if (this.opt.keys instanceof Array) {
        this._key = this.opt.keys[Math.floor(Math.random() * this.opt.keys.length)];
      } else {
        this._key = this.opt.keys;
      }
    }
  }

  _createClass(GaodeRoute, [{
    key: "key",
    get: function get() {
      return this._key;
    },
    set: function set(key) {
      this._key = key;
    }
    /**
    * 驾车路线规划
    * @param {Object} options  参数，可参考高德官网配置
    * @param {Array} options.origin  起点经纬度坐标
    * @param {Array} options.destination  终点经纬度坐标
    * @param {Array} options.avoidpolygons  避让区域坐标
    * @param {Function} success 成功后的回调函数 
    */

  }, {
    key: "queryDriving",
    value: function queryDriving(options, success, error) {
      options = options || {};

      if (!options.origin) {
        alert("缺少起点坐标！");
        return;
      }

      if (!options.destination) {
        alert("缺少终点坐标！");
        return;
      }

      var origin = options.origin;
      var gcj_origin = this.wgs2gcj(origin);
      gcj_origin = gcj_origin[0] + "," + gcj_origin[1];
      delete options.origin;
      var destination = options.destination;
      var gcj_destination = this.wgs2gcj(destination);
      gcj_destination = gcj_destination[0] + "," + gcj_destination[1];
      delete options.destination;
      var avoidpolygons = ''; // 避让区域

      if (options.avoidpolygons) {
        for (var i = 0; i < options.avoidpolygons.length; i++) {
          var avoidpolygon = options.avoidpolygons[i];
          var firstLnglat = '';
          var polygonstr = '';

          for (var j = 0; j < avoidpolygon.length; j++) {
            var lnglat = avoidpolygon[j];
            lnglat = this.wgs2gcj(lnglat);
            polygonstr += lnglat[0] + ',' + lnglat[1] + ',';
            if (j == 0) firstLnglat = lnglat[0] + ',' + lnglat[1];
          }

          polygonstr = polygonstr + firstLnglat;

          if (i == options.avoidpolygons.length - 1) {
            avoidpolygons += polygonstr;
          } else {
            avoidpolygons += polygonstr + '|';
          }
        }
      }

      delete options.avoidpolygons;
      var url = "https://restapi.amap.com/v5/direction/driving";
      var params = {
        key: this._key,
        origin: gcj_origin,
        destination: gcj_destination,
        strategy: 0,
        // 默认距离优先
        avoidpolygons: avoidpolygons,
        // 避让区不可超过81平方公里 避让去顶点不可超过16个
        show_fields: "polyline"
      };
      params = Object.assign(params, options || {});
      var that = this;
      axios.get(url, {
        params: params
      }).then(function (res) {
        if (res.status != 200 || res.data.infocode != '10000') {
          console.log("查询失败！");
          if (error) error(res.data);
          return;
        }

        var allroute = that.transformData(origin, destination, res.data.route);
        if (success) success(allroute);
      });
    }
  }, {
    key: "queryUndriving",
    value: function queryUndriving(url, options, success, error) {
      options = options || {};

      if (!options.origin) {
        alert("缺少起点坐标！");
        return;
      }

      if (!options.destination) {
        alert("缺少终点坐标！");
        return;
      }

      var origin = options.origin;
      var gcj_origin = this.wgs2gcj(origin);
      gcj_origin = origin[0] + "," + origin[1];
      delete options.origin;
      var destination = options.destination;
      var gcj_destination = this.wgs2gcj(destination);
      gcj_destination = destination[0] + "," + destination[1];
      delete options.destination;
      var params = {
        key: this._key,
        origin: gcj_origin,
        destination: gcj_destination,
        show_fields: "polyline"
      };
      params = Object.assign(params, options || {});
      var that = this;
      axios.get(url, {
        params: params
      }).then(function (res) {
        if (res.status != 200) {
          console.log("查询失败！");
          if (error) error(res.data);
          return;
        }

        var allroute = that.transformData(origin, destination, res.data.data || res.data.route);
        if (success) success(allroute);
      });
    } // 开始搜索

    /**
     * @param {Number} type 搜索类型（1、驾车路线；2、骑行路线；3、步行路线）
     * @param {Object} params 搜索参数
     * @param {Function} success 成功后的回调函数
     */

  }, {
    key: "query",
    value: function query(type, params, success, error) {
      type = Number(type || 1);
      var url = '';

      if (type == 1) {
        // 驾车
        this.queryDriving(params, success, error);
      } else if (type == 2) {
        // 骑行
        url = 'https://restapi.amap.com/v4/direction/bicycling';
        this.queryUndriving(url, params, success, error);
      } else if (type == 3) {
        // 步行
        url = 'https://restapi.amap.com/v3/direction/walking';
        this.queryUndriving(url, params, success, error);
      } else {
        alert("路径查询类型有误");
        return;
      }
    }
  }, {
    key: "transformWD",
    value: function transformWD(lng, lat) {
      var PI = 3.1415926535897932384626;
      var ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * Math.sqrt(Math.abs(lng));
      ret += (20.0 * Math.sin(6.0 * lng * PI) + 20.0 * Math.sin(2.0 * lng * PI)) * 2.0 / 3.0;
      ret += (20.0 * Math.sin(lat * PI) + 40.0 * Math.sin(lat / 3.0 * PI)) * 2.0 / 3.0;
      ret += (160.0 * Math.sin(lat / 12.0 * PI) + 320 * Math.sin(lat * PI / 30.0)) * 2.0 / 3.0;
      return ret;
    }
  }, {
    key: "transformJD",
    value: function transformJD(lng, lat) {
      var PI = 3.1415926535897932384626;
      var ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * Math.sqrt(Math.abs(lng));
      ret += (20.0 * Math.sin(6.0 * lng * PI) + 20.0 * Math.sin(2.0 * lng * PI)) * 2.0 / 3.0;
      ret += (20.0 * Math.sin(lng * PI) + 40.0 * Math.sin(lng / 3.0 * PI)) * 2.0 / 3.0;
      ret += (150.0 * Math.sin(lng / 12.0 * PI) + 300.0 * Math.sin(lng / 30.0 * PI)) * 2.0 / 3.0;
      return ret;
    }
  }, {
    key: "wgs2gcj",
    value: function wgs2gcj(arrdata) {
      var a = 6378245.0;
      var ee = 0.00669342162296594323;
      var PI = 3.1415926535897932384626;
      var lng = Number(arrdata[0]);
      var lat = Number(arrdata[1]);
      var dlat = this.transformWD(lng - 105.0, lat - 35.0);
      var dlng = this.transformJD(lng - 105.0, lat - 35.0);
      var radlat = lat / 180.0 * PI;
      var magic = Math.sin(radlat);
      magic = 1 - ee * magic * magic;
      var sqrtmagic = Math.sqrt(magic);
      dlat = dlat * 180.0 / (a * (1 - ee) / (magic * sqrtmagic) * PI);
      dlng = dlng * 180.0 / (a / sqrtmagic * Math.cos(radlat) * PI);
      var mglat = lat + dlat;
      var mglng = lng + dlng;
      mglng = Number(mglng.toFixed(6));
      mglat = Number(mglat.toFixed(6));
      return [mglng, mglat];
    }
  }, {
    key: "gcj2wgs",
    value: function gcj2wgs(arrdata) {
      var a = 6378245.0;
      var ee = 0.00669342162296594323;
      var PI = 3.1415926535897932384626;
      var lng = Number(arrdata[0]);
      var lat = Number(arrdata[1]);
      var dlat = this.transformWD(lng - 105.0, lat - 35.0);
      var dlng = this.transformJD(lng - 105.0, lat - 35.0);
      var radlat = lat / 180.0 * PI;
      var magic = Math.sin(radlat);
      magic = 1 - ee * magic * magic;
      var sqrtmagic = Math.sqrt(magic);
      dlat = dlat * 180.0 / (a * (1 - ee) / (magic * sqrtmagic) * PI);
      dlng = dlng * 180.0 / (a / sqrtmagic * Math.cos(radlat) * PI);
      var mglat = lat + dlat;
      var mglng = lng + dlng;
      var jd = lng * 2 - mglng;
      var wd = lat * 2 - mglat;
      jd = Number(jd.toFixed(6));
      wd = Number(wd.toFixed(6));
      return [jd, wd];
    }
  }, {
    key: "transformData",
    value: function transformData(start, end, route) {
      var _this = this;

      var paths = route.paths; // 所有路径

      var allroute = [];

      var _loop = function _loop(i) {
        var path = paths[i];
        var lnglats = [];
        var instructions = [];
        var distance = path.distance;

        for (var j = 0; j < path.steps.length; j++) {
          // 单个路径的坐标数组
          var item = path.steps[j];
          var instruction = path.steps[j].instruction;
          var polyline = item.polyline;
          polyline = polyline.split(";");
          polyline.forEach(function (element) {
            element = element.split(",");
            element = _this.gcj2wgs(element);
            lnglats.push([element[0], element[1]]);
          });
          instructions.push(instruction);
        } // 加上起点和终点


        lnglats.unshift(start);
        lnglats.push(end);
        allroute.push({
          lnglats: lnglats,
          instructions: instructions,
          distance: distance
        });
      };

      for (var i = 0; i < paths.length; i++) {
        _loop(i);
      }

      return allroute;
    }
  }]);

  return GaodeRoute;
}();

/* CompositeCoordinateSystem */

function CompositeCoordinateSystem(api) {
  this.dimensions = ['lng', 'lat'];
  this._mapOffset = [0, 0];
  this._api = api;
}

CompositeCoordinateSystem.dimensions = CompositeCoordinateSystem.prototype.dimensions = ['lng', 'lat'];

CompositeCoordinateSystem.prototype.setMapOffset = function (mapOffset) {
  this._mapOffset = mapOffset;
};

CompositeCoordinateSystem.prototype.getBMap = function () {
  return this.GLMap;
};

CompositeCoordinateSystem.prototype.dataToPoint = function (data) {
  var defVal = [99999, 99999];
  var position = Cesium.Cartesian3.fromDegrees(data[0], data[1]);

  if (!position) {
    return defVal;
  }

  var px = Cesium.SceneTransforms.wgs84ToWindowCoordinates(this.GLMap, position);

  if (!px) {
    return defVal;
  } //判断是否在球的背面


  var scene = this.GLMap;

  if (scene.mode === Cesium.SceneMode.SCENE3D) {
    var angle = Cesium.Cartesian3.angleBetween(scene.camera.position, position);
    if (angle > Cesium.Math.toRadians(80)) return [-100, -100];
  }

  return [px.x - this._mapOffset[0], px.y - this._mapOffset[1]];
};

CompositeCoordinateSystem.prototype.pointToData = function (pt) {
  var mapOffset = this._mapOffset;

  var pt = this._bmap.project([pt[0] + mapOffset[0], pt[1] + mapOffset[1]]);

  return [pt.lng, pt.lat];
};

CompositeCoordinateSystem.prototype.getViewRect = function () {
  var api = this._api;
  return new graphic$1.BoundingRect(0, 0, api.getWidth(), api.getHeight());
};

CompositeCoordinateSystem.prototype.getRoamTransform = function () {
  return matrix.create();
};

CompositeCoordinateSystem.create = function (ecModel, api) {
  var GLMap = CompositeCoordinateSystem.prototype.GLMap;
  var coordSys;
  ecModel.eachComponent('GLMap', function (GLMapModel) {
    coordSys = new CompositeCoordinateSystem(GLMap, api);
    coordSys.setMapOffset(GLMapModel.__mapOffset || [0, 0]);
    GLMapModel.coordinateSystem = coordSys;
  });
  ecModel.eachSeries(function (seriesModel) {
    if (seriesModel.get('coordinateSystem') === 'GLMap') {
      coordSys = new CompositeCoordinateSystem(GLMap, api);
      seriesModel.coordinateSystem = new CompositeCoordinateSystem(GLMap, api);
    }
  });
};

function registerGLMap(viewer) {
  CompositeCoordinateSystem.prototype.GLMap = viewer.scene;
  registerCoordinateSystem('GLMap', CompositeCoordinateSystem);
  registerAction({
    type: 'GLMapRoam',
    event: 'GLMapRoam',
    update: 'updateLayout'
  }, function (payload, ecModel) {});
  /* CompositeMapModel */

  extendComponentModel({
    type: 'GLMap',
    getBMap: function getBMap() {
      return this._GLMap;
    },
    defaultOption: {
      roam: false
    }
  });
  /* 	CompositeMapView */

  extendComponentView({
    type: 'GLMap',
    lastCenter: null,
    lastCameraPitch: null,
    isControl: true,
    init: function init(ecModel, api) {
      var that = this;

      var moveHandler = function moveHandler(type, target) {
        api.dispatchAction({
          type: 'GLMapRoam'
        });
      }; //绑定渲染事件 实时监控echarts div和地球的位置


      var handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
      viewer.scene.postRender.addEventListener(function () {
        var cameraPosition = viewer.camera.position;

        if (!cameraPosition) {
          that.isControl = true;
          return;
        }

        var height = Cesium.Cartographic.fromCartesian(cameraPosition).height;

        if (height > 450) {
          that.isControl = true;
        }

        if (that.isControl) moveHandler();
      });
      handler.setInputAction(function (evt) {
        var cameraPosition = viewer.camera.position;

        if (!cameraPosition) {
          that.isControl = true;
          return;
        }

        var height = Cesium.Cartographic.fromCartesian(cameraPosition).height;

        if (height < 450) {
          that.isControl = false;
        }
      }, Cesium.ScreenSpaceEventType.MIDDLE_DOWN);
      handler.setInputAction(function (evt) {
        that.isControl = true;
      }, Cesium.ScreenSpaceEventType.LEFT_DOWN);
    },
    render: function render(GLMapModel, ecModel, api) {},
    dispose: function dispose(target) {
      viewer.scene.postRender.removeEventListener(this.moveHandler, this);
    },
    getCenter: function getCenter(viewer) {
      var canvas = viewer.scene.canvas;
      var center = new Cesium.Cartesian2(canvas.clientWidth / 2, canvas.clientHeight / 2);
      var ray = viewer.scene.camera.getPickRay(center);
      var center = viewer.scene.globe.pick(ray, viewer.scene, new Cesium.Cartesian3());
      return center;
    }
  });
}

function echartMap(viewer, option) {
  // 创建echarts的div
  var chartContainer = document.createElement('div');
  var scene = viewer.scene;
  scene.canvas.setAttribute('tabIndex', 0);
  chartContainer.style.position = 'absolute';
  chartContainer.style.top = '0px';
  chartContainer.style.left = '0px';
  chartContainer.style.width = scene.canvas.width + 'px';
  chartContainer.style.height = scene.canvas.height + 'px';
  chartContainer.style.pointerEvents = 'none'; //控制echarts是否可以点击

  chartContainer.setAttribute('id', 'echarts_div');
  chartContainer.setAttribute('class', 'echartsLayer');
  viewer.container.appendChild(chartContainer); // 注册坐标系统

  registerGLMap(viewer); // 数据赋值

  var overlay = init(chartContainer);
  overlay.setOption(option);
}

var baiduMapLayer = mapv$1 ? baiduMapLayer$1 : null;
var BaseLayer$1 = baiduMapLayer ? baiduMapLayer.__proto__ : Function;
var backAngle = Cesium.Math.toRadians(75);

var MapVRenderer = /*#__PURE__*/function (_BaseLayer) {
  _inherits(MapVRenderer, _BaseLayer);

  var _super = _createSuper(MapVRenderer);

  function MapVRenderer(t, e, i, n) {
    var _this;

    _classCallCheck(this, MapVRenderer);

    if (_this = _super.call(this, t, e, i), BaseLayer$1) {
      _this.map = t, _this.scene = t.scene, _this.dataSet = e;
      i = i || {}, _this.init(i), _this.argCheck(i), _this.initDevicePixelRatio(), _this.canvasLayer = n, _this.stopAniamation = !1, _this.animation = i.animation, _this.clickEvent = _this.clickEvent.bind(_assertThisInitialized(_this)), _this.mousemoveEvent = _this.mousemoveEvent.bind(_assertThisInitialized(_this)), _this.bindEvent();
    }

    return _possibleConstructorReturn(_this);
  }

  _createClass(MapVRenderer, [{
    key: "initDevicePixelRatio",
    value: function initDevicePixelRatio() {
      this.devicePixelRatio = window.devicePixelRatio || 1;
    }
  }, {
    key: "clickEvent",
    value: function clickEvent(t) {
      var e = t.point;

      _get(_getPrototypeOf(MapVRenderer.prototype), "clickEvent", this).call(this, e, t);
    }
  }, {
    key: "mousemoveEvent",
    value: function mousemoveEvent(t) {
      var e = t.point;

      _get(_getPrototypeOf(MapVRenderer.prototype), "mousemoveEvent", this).call(this, e, t);
    }
  }, {
    key: "addAnimatorEvent",
    value: function addAnimatorEvent() {}
  }, {
    key: "animatorMovestartEvent",
    value: function animatorMovestartEvent() {
      var t = this.options.animation;
      this.isEnabledTime() && this.animator && (this.steps.step = t.stepsRange.start);
    }
  }, {
    key: "animatorMoveendEvent",
    value: function animatorMoveendEvent() {
      this.isEnabledTime() && this.animator;
    }
  }, {
    key: "bindEvent",
    value: function bindEvent() {
      this.map;
      this.options.methods && (this.options.methods.click, this.options.methods.mousemove);
    }
  }, {
    key: "unbindEvent",
    value: function unbindEvent() {
      var t = this.map;
      this.options.methods && (this.options.methods.click && t.off("click", this.clickEvent), this.options.methods.mousemove && t.off("mousemove", this.mousemoveEvent));
    }
  }, {
    key: "getContext",
    value: function getContext() {
      return this.canvasLayer.canvas.getContext(this.context);
    }
  }, {
    key: "init",
    value: function init(t) {
      this.options = t, this.initDataRange(t), this.context = this.options.context || "2d", this.options.zIndex && this.canvasLayer && this.canvasLayer.setZIndex(this.options.zIndex), this.initAnimator();
    }
  }, {
    key: "_canvasUpdate",
    value: function _canvasUpdate(t) {
      this.map;
      var e = this.scene;

      if (this.canvasLayer && !this.stopAniamation) {
        var i = this.options.animation,
            n = this.getContext();

        if (this.isEnabledTime()) {
          if (void 0 === t) return void this.clear(n);
          "2d" === this.context && (n.save(), n.globalCompositeOperation = "destination-out", n.fillStyle = "rgba(0, 0, 0, .1)", n.fillRect(0, 0, n.canvas.width, n.canvas.height), n.restore());
        } else this.clear(n);

        if ("2d" === this.context) for (var o in this.options) {
          n[o] = this.options[o];
        } else n.clear(n.COLOR_BUFFER_BIT);
        var a = {
          transferCoordinate: function transferCoordinate(t) {
            var defVal = [99999, 99999]; //坐标转换

            var position = Cesium.Cartesian3.fromDegrees(t[0], t[1]);

            if (!position) {
              return defVal;
            }

            var px = e.cartesianToCanvasCoordinates(position);

            if (!px) {
              return defVal;
            } //判断是否在球的背面  


            var angle = Cesium.Cartesian3.angleBetween(e.camera.position, position);
            if (angle > backAngle) return false; //判断是否在球的背面

            return [px.x, px.y];
          }
        };
        void 0 !== t && (a.filter = function (e) {
          var n = i.trails || 10;
          return !!(t && e.time > t - n && e.time < t);
        });
        var c = this.dataSet.get(a);
        this.processData(c), "m" == this.options.unit && this.options.size, this.options._size = this.options.size;
        var h = Cesium.SceneTransforms.wgs84ToWindowCoordinates(e, Cesium.Cartesian3.fromDegrees(0, 0));
        this.drawContext(n, new DataSet(c), this.options, h), this.options.updateCallback && this.options.updateCallback(t);
      }
    }
  }, {
    key: "updateData",
    value: function updateData(t, e) {
      var i = t;
      i && i.get && (i = i.get()), void 0 != i && this.dataSet.set(i), _get(_getPrototypeOf(MapVRenderer.prototype), "update", this).call(this, {
        options: e
      });
    }
  }, {
    key: "addData",
    value: function addData(t, e) {
      var i = t;
      t && t.get && (i = t.get()), this.dataSet.add(i), this.update({
        options: e
      });
    }
  }, {
    key: "getData",
    value: function getData() {
      return this.dataSet;
    }
  }, {
    key: "removeData",
    value: function removeData(t) {
      if (this.dataSet) {
        var e = this.dataSet.get({
          filter: function filter(e) {
            return null == t || "function" != typeof t || !t(e);
          }
        });
        this.dataSet.set(e), this.update({
          options: null
        });
      }
    }
  }, {
    key: "clearData",
    value: function clearData() {
      this.dataSet && this.dataSet.clear(), this.update({
        options: null
      });
    }
  }, {
    key: "draw",
    value: function draw() {
      this.canvasLayer.draw();
    }
  }, {
    key: "clear",
    value: function clear(t) {
      t && t.clearRect && t.clearRect(0, 0, t.canvas.width, t.canvas.height);
    }
  }]);

  return MapVRenderer;
}(BaseLayer$1);

var h = 0;

var MapvLayer = /*#__PURE__*/function () {
  function MapvLayer(t, e, i, n) {
    _classCallCheck(this, MapvLayer);

    this.map = t, this.scene = t.scene, this.mapvBaseLayer = new MapVRenderer(t, e, i, this), this.mapVOptions = i, this.initDevicePixelRatio(), this.canvas = this._createCanvas(), this.render = this.render.bind(this), void 0 != n ? (this.container = n, n.appendChild(this.canvas)) : (this.container = t.container, this.addInnerContainer()), this.bindEvent(), this._reset();
  }

  _createClass(MapvLayer, [{
    key: "initDevicePixelRatio",
    value: function initDevicePixelRatio() {
      this.devicePixelRatio = window.devicePixelRatio || 1;
    }
  }, {
    key: "addInnerContainer",
    value: function addInnerContainer() {
      this.container.appendChild(this.canvas);
    }
  }, {
    key: "bindEvent",
    value: function bindEvent() {
      //绑定cesium事件与mapv联动
      this.innerMoveStart = this.moveStartEvent.bind(this), this.innerMoveEnd = this.moveEndEvent.bind(this);
      this.scene.camera.moveStart.addEventListener(this.innerMoveStart, this);
      this.scene.camera.moveEnd.addEventListener(this.innerMoveEnd, this);
    }
  }, {
    key: "unbindEvent",
    value: function unbindEvent() {
      this.scene.camera.moveStart.removeEventListener(this.innerMoveStart, this);
      this.scene.camera.moveEnd.removeEventListener(this.innerMoveEnd, this);
      this.scene.postRender.removeEventListener(this._reset, this);
    }
  }, {
    key: "moveStartEvent",
    value: function moveStartEvent() {
      this.mapvBaseLayer && this.mapvBaseLayer.animatorMovestartEvent(); //, this._unvisiable()

      this.scene.postRender.addEventListener(this._reset, this);
    }
  }, {
    key: "moveEndEvent",
    value: function moveEndEvent() {
      this.mapvBaseLayer && this.mapvBaseLayer.animatorMoveendEvent(), this._reset(); //, this._visiable()

      this.scene.postRender.removeEventListener(this._reset, this);
    }
  }, {
    key: "zoomStartEvent",
    value: function zoomStartEvent() {
      this._unvisiable();
    }
  }, {
    key: "zoomEndEvent",
    value: function zoomEndEvent() {
      this._unvisiable();
    }
  }, {
    key: "addData",
    value: function addData(t, e) {
      void 0 != this.mapvBaseLayer && this.mapvBaseLayer.addData(t, e);
    }
  }, {
    key: "updateData",
    value: function updateData(t, e) {
      void 0 != this.mapvBaseLayer && this.mapvBaseLayer.updateData(t, e);
    }
  }, {
    key: "getData",
    value: function getData() {
      return this.mapvBaseLayer && (this.dataSet = this.mapvBaseLayer.getData()), this.dataSet;
    }
  }, {
    key: "removeData",
    value: function removeData(t) {
      void 0 != this.mapvBaseLayer && this.mapvBaseLayer && this.mapvBaseLayer.removeData(t);
    }
  }, {
    key: "removeAllData",
    value: function removeAllData() {
      void 0 != this.mapvBaseLayer && this.mapvBaseLayer.clearData();
    }
  }, {
    key: "_visiable",
    value: function _visiable() {
      return this.canvas.style.display = "block";
    }
  }, {
    key: "_unvisiable",
    value: function _unvisiable() {
      return this.canvas.style.display = "none";
    }
  }, {
    key: "_createCanvas",
    value: function _createCanvas() {
      var t = document.createElement("canvas");
      t.id = this.mapVOptions.layerid || "mapv" + h++, t.style.position = "absolute", t.style.top = "0px", t.style.left = "0px", t.style.pointerEvents = "none", t.style.zIndex = this.mapVOptions.zIndex || 100, t.width = parseInt(this.map.canvas.width), t.height = parseInt(this.map.canvas.height), t.style.width = this.map.canvas.style.width, t.style.height = this.map.canvas.style.height;
      var e = this.devicePixelRatio;
      return "2d" == this.mapVOptions.context && t.getContext(this.mapVOptions.context).scale(e, e), t;
    }
  }, {
    key: "_reset",
    value: function _reset() {
      this.resizeCanvas(), this.fixPosition(), this.onResize(), this.render();
    }
  }, {
    key: "draw",
    value: function draw() {
      this._reset();
    }
  }, {
    key: "show",
    value: function show() {
      this._visiable();
    }
  }, {
    key: "hide",
    value: function hide() {
      this._unvisiable();
    }
  }, {
    key: "destroy",
    value: function destroy() {
      this.remove();
    }
  }, {
    key: "remove",
    value: function remove() {
      void 0 != this.mapvBaseLayer && (this.removeAllData(), this.mapvBaseLayer.clear(this.mapvBaseLayer.getContext()), this.mapvBaseLayer = void 0, this.canvas.parentElement.removeChild(this.canvas));
    }
  }, {
    key: "update",
    value: function update(t) {
      void 0 != t && this.updateData(t.data, t.options);
    }
  }, {
    key: "resizeCanvas",
    value: function resizeCanvas() {
      if (void 0 != this.canvas && null != this.canvas) {
        var t = this.canvas;
        t.style.position = "absolute", t.style.top = "0px", t.style.left = "0px", t.width = parseInt(this.map.canvas.width), t.height = parseInt(this.map.canvas.height), t.style.width = this.map.canvas.style.width, t.style.height = this.map.canvas.style.height;
      }
    }
  }, {
    key: "fixPosition",
    value: function fixPosition() {}
  }, {
    key: "onResize",
    value: function onResize() {}
  }, {
    key: "render",
    value: function render() {
      void 0 != this.mapvBaseLayer && this.mapvBaseLayer._canvasUpdate();
    }
  }]);

  return MapvLayer;
}();

function mapv (viewer, dataSet1, options1) {
  return new MapvLayer(viewer, dataSet1, options1);
}

/**
 * @description 二维热力图类，基于h337类扩展
 * @class
 */

var Heatmap = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础参数
   * @param {Array} opt.list 热力值数组
   * @param {Array} opt.raduis 热力点半径
   * @param {Array} opt.gradient 颜色配置
   */
  function Heatmap(viewer, opt) {
    _classCallCheck(this, Heatmap);

    this.viewer = viewer;
    this.opt = opt || {};
    this.list = this.opt.list || [];

    if (!this.list || this.list.length < 2) {
      console.log("热力图点位不得少于3个！");
      return;
    }
    /**
     *@property {Cesium.Entity} polygon 热力图面
     */


    this.polygon = undefined;
    this.dom = undefined;
    this.id = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    this.canvasw = 200;
    this.createDom();
    var config = {
      container: document.getElementById("vis3d-heatmap-".concat(this.id)),
      radius: this.opt.raduis || 20,
      backgroundColor: 'rgba(0,0,0,1)',
      maxOpacity: this.opt.maxOpacity || .7,
      minOpacity: 0.01,
      blur: this.opt.blur || .75,
      gradient: this.opt.gradient || {
        '.1': 'blue',
        '.5': 'yellow',
        '.7': 'red',
        '.99': 'white'
      }
    };
    this.heatmapInstance = h337.create(config);
    this.init();
  }

  _createClass(Heatmap, [{
    key: "init",
    value: function init() {
      this.hierarchy = [];

      for (var ind = 0; ind < this.list.length; ind++) {
        var position = Cesium.Cartesian3.fromDegrees(this.list[ind].lnglat[0], this.list[ind].lnglat[1]);
        this.hierarchy.push(position);
      }

      this.polygon = undefined;
      var bound = this.getBound(this.hierarchy);
      if (!bound) return;
      var points = [];
      var x_axios = Cesium.Cartesian3.subtract(bound.rightTop, bound.leftTop, new Cesium.Cartesian3());
      x_axios = Cesium.Cartesian3.normalize(x_axios, new Cesium.Cartesian3());
      var y_axios = Cesium.Cartesian3.subtract(bound.leftBottom, bound.leftTop, new Cesium.Cartesian3());
      y_axios = Cesium.Cartesian3.normalize(y_axios, new Cesium.Cartesian3());
      var girthX = Cesium.Cartesian3.distance(bound.rightTop, bound.leftTop);
      var girthY = Cesium.Cartesian3.distance(bound.leftBottom, bound.leftTop);

      for (var i = 0; i < this.hierarchy.length; i++) {
        var p1 = this.hierarchy[i];
        var p_origin = Cesium.Cartesian3.subtract(p1, bound.leftTop, new Cesium.Cartesian3());
        var diffX = Cesium.Cartesian3.dot(p_origin, x_axios);
        var diffY = Cesium.Cartesian3.dot(p_origin, y_axios);
        points.push({
          x: Number(diffX / girthX * this.canvasw).toFixed(0),
          y: Number(diffY / girthY * this.canvasw).toFixed(0),
          value: this.list[i].value
        });
      }

      this.heatmapInstance.setData({
        max: 100,
        data: points
      });
      this.createPolygon([bound.leftTop, bound.leftBottom, bound.rightBottom, bound.rightTop]);
    }
  }, {
    key: "createPolygon",
    value: function createPolygon(positions) {
      this.polygon = this.viewer.entities.add({
        polygon: {
          hierarchy: new Cesium.PolygonHierarchy(positions),
          material: this.heatmapInstance.getDataURL(),
          heightReference: 1
        }
      });
      if (this.opt.zoomTo) this.viewer.zoomTo(this.polygon);
    }
  }, {
    key: "createProvider",
    value: function createProvider() {}
  }, {
    key: "createDom",
    value: function createDom() {
      this.dom = window.document.createElement("div");
      this.dom.id = "vis3d-heatmap-".concat(this.id);
      this.dom.className = "vis3d-heatmap";
      this.dom.style.width = this.canvasw + "px";
      this.dom.style.height = this.canvasw + "px";
      this.dom.style.position = "absolute";
      this.dom.style.display = "none";
      var mapDom = window.document.getElementById(this.viewer.container.id);
      mapDom.appendChild(this.dom);
    }
    /**
     * 销毁
     */

  }, {
    key: "destory",
    value: function destory() {
      var dom = document.getElementById("vis3d-heatmap-".concat(this.id));
      if (dom) dom.remove();

      if (this.polygon) {
        this.viewer.entities.remove(this.polygon);
        this.polygon = undefined;
      }
    } // 扩展边界 防止出现热力图被分割

  }, {
    key: "getBound",
    value: function getBound(positions) {
      var rect = this.toRectangle(positions); // 转为正方形

      var lnglats = util$1.cartesiansToLnglats(rect, this.viewer);
      var minLat = Number.MAX_VALUE,
          maxLat = Number.MIN_VALUE,
          minLng = Number.MAX_VALUE,
          maxLng = Number.MIN_VALUE;
      var length = rect.length;

      for (var i = 0; i < length; i++) {
        var lnglat = lnglats[i];

        if (lnglat[0] < minLng) {
          minLng = lnglat[0];
        }

        if (lnglat[0] > maxLng) {
          maxLng = lnglat[0];
        }

        if (lnglat[1] < minLat) {
          minLat = lnglat[1];
        }

        if (lnglat[1] > maxLat) {
          maxLat = lnglat[1];
        }
      }

      var diff_lat = maxLat - minLat;
      var diff_lng = maxLng - minLng;
      minLat = minLat - diff_lat / length;
      maxLat = maxLat + diff_lat / length;
      minLng = minLng - diff_lng / length;
      maxLng = maxLng + diff_lng / length;
      return {
        leftTop: Cesium.Cartesian3.fromDegrees(minLng, maxLat),
        leftBottom: Cesium.Cartesian3.fromDegrees(minLng, minLat),
        rightTop: Cesium.Cartesian3.fromDegrees(maxLng, maxLat),
        rightBottom: Cesium.Cartesian3.fromDegrees(maxLng, minLat)
      };
    } // 任何图形均转化为正方形

  }, {
    key: "toRectangle",
    value: function toRectangle(hierarchy) {
      if (!hierarchy) return;
      var boundingSphere = Cesium.BoundingSphere.fromPoints(hierarchy, new Cesium.BoundingSphere());
      var center = boundingSphere.center;
      var radius = boundingSphere.radius;
      var modelMatrix = Cesium.Transforms.eastNorthUpToFixedFrame(center.clone());
      var modelMatrix_inverse = Cesium.Matrix4.inverse(modelMatrix.clone(), new Cesium.Matrix4());
      var roate_y = new Cesium.Cartesian3(0, 1, 0);
      var arr = [];

      for (var i = 45; i <= 360; i += 90) {
        var roateZ_mtx = Cesium.Matrix3.fromRotationZ(Cesium.Math.toRadians(i), new Cesium.Matrix3());
        var yaix_roate = Cesium.Matrix3.multiplyByVector(roateZ_mtx, roate_y, new Cesium.Cartesian3());
        yaix_roate = Cesium.Cartesian3.normalize(yaix_roate, new Cesium.Cartesian3());
        var third = Cesium.Cartesian3.multiplyByScalar(yaix_roate, radius, new Cesium.Cartesian3());
        var poi = Cesium.Matrix4.multiplyByPoint(modelMatrix, third.clone(), new Cesium.Cartesian3());
        arr.push(poi);
      }

      return arr;
    }
  }]);

  return Heatmap;
}();

/**
 * @description 三维热力图类，基于h337类扩展
 * @class
 */

var Heatmap3d = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 地图viewer对象 
   * @param {Object} opt 基础参数
   * @param {Array} opt.list 热力值数组
   * @param {Array} opt.raduis 热力点半径
   * @param {Array} opt.baseHeight 最低高度 
   * @param {Array} opt.gradient 颜色配置
   */
  function Heatmap3d(viewer, opt) {
    _classCallCheck(this, Heatmap3d);

    this.viewer = viewer;
    this.opt = opt || {};
    this.list = this.opt.list || [];

    if (!this.list || this.list.length < 2) {
      console.log("热力图点位不得少于3个！");
      return;
    }

    this.dom = undefined;
    this.id = Number(new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0));
    this.canvasw = 200;
    this.bound = undefined; // 四角坐标

    this.rect = {}; // 经纬度范围

    this.x_axios = undefined; // x 轴

    this.y_axios = undefined; // y 轴

    this.girthX = 0; // x轴长度

    this.girthY = 0; // y轴长度

    this.baseHeight = this.opt.baseHeight || 0;
    this.createDom();
    var config = {
      container: document.getElementById("vis3d-heatmap-".concat(this.id)),
      radius: this.opt.raduis || 20,
      maxOpacity: .7,
      minOpacity: 0,
      blur: .75,
      gradient: this.opt.gradient || {
        '.1': 'blue',
        '.5': 'yellow',
        '.7': 'red',
        '.99': 'white'
      }
    };
    this.primitiveType = opt.primitiveType || "TRIANGLES";
    this.heatmapInstance = h337.create(config);
    /**
     *@property {Cesium.Primitive} primitive 热力图图元
     */

    this.primitive = undefined;
    this.init();
    if (this.opt.moueseMove) this.bindTooltip();
  }

  _createClass(Heatmap3d, [{
    key: "init",
    value: function init() {
      this.hierarchy = [];

      for (var ind = 0; ind < this.list.length; ind++) {
        var position = Cesium.Cartesian3.fromDegrees(this.list[ind].lnglat[0], this.list[ind].lnglat[1], 0);
        this.hierarchy.push(position);
      }

      this.computeBound(this.hierarchy);
      var points = [];

      for (var i = 0; i < this.hierarchy.length; i++) {
        var p1 = this.hierarchy[i];
        var rete = this.computeRateInBound(p1);
        points.push({
          x: rete.x,
          y: rete.y,
          value: this.list[i].value
        });
      }

      this.heatmapInstance.addData(points);
      var instance = new Cesium.GeometryInstance({
        geometry: this.createGeometry()
      });
      this.primitive = this.viewer.scene.primitives.add(new Cesium.Primitive({
        geometryInstances: instance,
        appearance: new Cesium.MaterialAppearance({
          material: new Cesium.Material({
            fabric: {
              type: 'Image',
              uniforms: {
                image: this.heatmapInstance.getDataURL()
              }
            }
          }),
          translucent: true,
          flat: true
        }),
        asynchronous: false
      }));
      this.primitive.id = "heatmap3d";
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      var dom = document.getElementById("vis3d-heatmap-".concat(this.id));
      if (dom) dom.remove();

      if (this.primitive) {
        this.viewer.scene.primitives.remove(this.primitive);
        this.primitive = undefined;
      }
    } // 计算当前坐标在范围中位置 换算为canvas中的像素坐标

  }, {
    key: "computeRateInBound",
    value: function computeRateInBound(position) {
      if (!position) return;
      var ctgc = Cesium.Cartographic.fromCartesian(position.clone());
      ctgc.height = 0;
      position = Cesium.Cartographic.toCartesian(ctgc.clone());
      var p_origin = Cesium.Cartesian3.subtract(position.clone(), this.bound.leftTop, new Cesium.Cartesian3());
      var diffX = Cesium.Cartesian3.dot(p_origin, this.x_axios);
      var diffY = Cesium.Cartesian3.dot(p_origin, this.y_axios);
      return {
        x: Number(diffX / this.girthX * this.canvasw).toFixed(0),
        y: Number(diffY / this.girthY * this.canvasw).toFixed(0)
      };
    }
  }, {
    key: "computeBound",
    value: function computeBound(positions) {
      // 先转化为正方形
      if (!positions) return;
      var boundingSphere = Cesium.BoundingSphere.fromPoints(positions, new Cesium.BoundingSphere());
      var center = boundingSphere.center;
      var radius = boundingSphere.radius;
      var modelMatrix = Cesium.Transforms.eastNorthUpToFixedFrame(center.clone());
      var modelMatrix_inverse = Cesium.Matrix4.inverse(modelMatrix.clone(), new Cesium.Matrix4());
      var roate_y = new Cesium.Cartesian3(0, 1, 0);
      var rect = [];

      for (var i = 45; i <= 360; i += 90) {
        var roateZ_mtx = Cesium.Matrix3.fromRotationZ(Cesium.Math.toRadians(i), new Cesium.Matrix3());
        var yaix_roate = Cesium.Matrix3.multiplyByVector(roateZ_mtx, roate_y, new Cesium.Cartesian3());
        yaix_roate = Cesium.Cartesian3.normalize(yaix_roate, new Cesium.Cartesian3());
        var third = Cesium.Cartesian3.multiplyByScalar(yaix_roate, radius, new Cesium.Cartesian3());
        var poi = Cesium.Matrix4.multiplyByPoint(modelMatrix, third.clone(), new Cesium.Cartesian3());
        rect.push(poi);
      }

      var lnglats = util$1.cartesiansToLnglats(rect, this.viewer);
      var minLat = Number.MAX_VALUE,
          maxLat = Number.MIN_VALUE,
          minLng = Number.MAX_VALUE,
          maxLng = Number.MIN_VALUE;
      var length = rect.length;

      for (var _i = 0; _i < length; _i++) {
        var lnglat = lnglats[_i];

        if (lnglat[0] < minLng) {
          minLng = lnglat[0];
        }

        if (lnglat[0] > maxLng) {
          maxLng = lnglat[0];
        }

        if (lnglat[1] < minLat) {
          minLat = lnglat[1];
        }

        if (lnglat[1] > maxLat) {
          maxLat = lnglat[1];
        }
      }

      var diff_lat = maxLat - minLat;
      var diff_lng = maxLng - minLng; // 放大正方形轮廓

      this.rect.minLat = minLat - diff_lat / length;
      this.rect.maxLat = maxLat + diff_lat / length;
      this.rect.minLng = minLng - diff_lng / length;
      this.rect.maxLng = maxLng + diff_lng / length;
      this.bound = {
        leftTop: Cesium.Cartesian3.fromDegrees(this.rect.minLng, this.rect.maxLat),
        leftBottom: Cesium.Cartesian3.fromDegrees(this.rect.minLng, this.rect.minLat),
        rightTop: Cesium.Cartesian3.fromDegrees(this.rect.maxLng, this.rect.maxLat),
        rightBottom: Cesium.Cartesian3.fromDegrees(this.rect.maxLng, this.rect.minLat)
      };
      this.x_axios = Cesium.Cartesian3.subtract(this.bound.rightTop, this.bound.leftTop, new Cesium.Cartesian3());
      this.x_axios = Cesium.Cartesian3.normalize(this.x_axios, new Cesium.Cartesian3());
      this.y_axios = Cesium.Cartesian3.subtract(this.bound.leftBottom, this.bound.leftTop, new Cesium.Cartesian3());
      this.y_axios = Cesium.Cartesian3.normalize(this.y_axios, new Cesium.Cartesian3());
      this.girthX = Cesium.Cartesian3.distance(this.bound.rightTop, this.bound.leftTop);
      this.girthY = Cesium.Cartesian3.distance(this.bound.leftBottom, this.bound.leftTop);
    }
  }, {
    key: "createGeometry",
    value: function createGeometry() {
      var opt = this.getGrain();
      var geometry = new Cesium.Geometry({
        attributes: new Cesium.GeometryAttributes({
          position: new Cesium.GeometryAttribute({
            componentDatatype: Cesium.ComponentDatatype.DOUBLE,
            componentsPerAttribute: 3,
            values: opt.positions
          }),
          st: new Cesium.GeometryAttribute({
            componentDatatype: Cesium.ComponentDatatype.FLOAT,
            componentsPerAttribute: 2,
            values: new Float32Array(opt.st)
          })
        }),
        indices: new Uint16Array(opt.indices),
        primitiveType: Cesium.PrimitiveType[this.primitiveType],
        boundingSphere: Cesium.BoundingSphere.fromVertices(opt.positions)
      });
      return geometry;
    } // 根据经纬度跨度和canvas的宽高 来计算顶点坐标及顶点法向量

  }, {
    key: "getGrain",
    value: function getGrain(opt) {
      var canvasW = this.canvasW || 200;
      var canvasH = this.canvasW || 200;
      var maxLng = this.rect.maxLng;
      var maxLat = this.rect.maxLat;
      var minLng = this.rect.minLng;
      var minLat = this.rect.minLat;
      var granLng_w = (maxLng - minLng) / canvasW; // 经度粒度

      var granLat_H = (maxLat - minLat) / canvasH; // 经度粒度

      var positions = [];
      var st = [];
      var indices = [];

      for (var i = 0; i < canvasW; i++) {
        var nowLng = minLng + granLng_w * i;

        for (var j = 0; j < canvasH; j++) {
          var nowLat = minLat + granLat_H * j;
          var value = this.heatmapInstance.getValueAt({
            x: i,
            y: j
          });
          var cartesian3 = Cesium.Cartesian3.fromDegrees(nowLng, nowLat, this.baseHeight + value);
          positions.push(cartesian3.x, cartesian3.y, cartesian3.z);
          st.push(i / canvasW, j / canvasH);

          if (j != canvasH - 1 && i != canvasW - 1) {
            indices.push(i * canvasH + j, i * canvasH + j + 1, (i + 1) * canvasH + j);
            indices.push((i + 1) * canvasH + j, (i + 1) * canvasH + j + 1, i * canvasH + j + 1);
          }
        }
      }

      return {
        positions: positions,
        st: st,
        indices: indices
      };
    }
  }, {
    key: "createDom",
    value: function createDom() {
      this.dom = window.document.createElement("div");
      this.dom.id = "vis3d-heatmap-".concat(this.id);
      this.dom.className = "vis3d-heatmap";
      this.dom.style.width = this.canvasw + "px";
      this.dom.style.height = this.canvasw + "px";
      this.dom.style.position = "absolute";
      this.dom.style.display = "none";
      var mapDom = window.document.getElementById(this.viewer.container.id);
      mapDom.appendChild(this.dom);
    }
  }, {
    key: "bindTooltip",
    value: function bindTooltip() {
      var that = this;
      if (!this.popupHandler) this.popupHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.popupHandler.setInputAction(function (evt) {
        //单击开始绘制
        var pick = that.viewer.scene.pick(evt.endPosition);

        if (!pick || !pick.primitive || pick.primitive.id != 'heatmap3d') {
          if (that.opt.moueseMove) that.opt.moueseMove(undefined);
        } else {
          var position = that.viewer.scene.pickPosition(evt.endPosition);
          var rate = that.computeRateInBound(position);
          console.log("rate--->", rate);
          var value = that.heatmapInstance.getValueAt(rate);
          if (that.opt.moueseMove) that.opt.moueseMove({
            value: value,
            px: evt.endPosition,
            position: position
          });
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }]);

  return Heatmap3d;
}();

var Cesium$b = window.Cesium;

function r(t, e) {
  var i = t,
      n = e,
      r = Math.cos,
      o = Math.sin;
  return [r(-i) * r(n), o(-i) * r(n), o(n)];
}

function o(t, e, i, n) {
  return t + n / i * (e - t);
}

function a$2(t, e) {
  var i = e.findIndex(function (e) {
    return e.fov > t;
  });

  if (i > 0) {
    var n = e[i - 1],
        r = e[i],
        o = (t - n.fov) / (r.fov - n.fov);
    return n.radius * (1 - o) + r.radius * o;
  }
}

function s(t, e, i, n, s, u, l) {
  for (var h = new Float32Array((s + 1) * (u + 1) * 3), d = 0; d < s + 1; ++d) {
    for (var m = 0; m < u + 1; ++m) {
      var f = o(i, n, u, m),
          p = r(o(t, e, s, d), f),
          c = l ? a$2(f, l) : 1;
      h[3 * (m * (s + 1) + d) + 0] = p[0] * c, h[3 * (m * (s + 1) + d) + 1] = p[1] * c, h[3 * (m * (s + 1) + d) + 2] = p[2] * c;
    }
  }

  return h;
}

function u(t, e, i, n, s, u, l) {
  for (var h = new Float32Array((n + 1) * (s + 1) * 3), d = 0; d < n + 1; ++d) {
    for (var m = 0; m < s + 1; ++m) {
      var f = o(e, i, s, m),
          p = r(t, f),
          c = u ? a$2(f, u) : 1,
          _ = l ? a$2(f, l) : 1,
          g = o(c, _, n, d);

      h[3 * (m * (n + 1) + d) + 0] = p[0] * g, h[3 * (m * (n + 1) + d) + 1] = p[1] * g, h[3 * (m * (n + 1) + d) + 2] = p[2] * g;
    }
  }

  return h;
}

function l$1(t, e) {
  for (var i = new Uint16Array(t * e * 6), n = 0; n < t; ++n) {
    for (var r = 0; r < e; ++r) {
      var o = r * (t + 1) + n,
          a = r * (t + 1) + n + 1,
          s = (r + 1) * (t + 1) + n,
          u = (r + 1) * (t + 1) + n + 1,
          l = 6 * (r * t + n);
      i[l + 0] = o, i[l + 1] = a, i[l + 2] = u, i[l + 3] = o, i[l + 4] = u, i[l + 5] = s;
    }
  }

  return i;
}

function h$1(t, e, i, n) {
  for (var r = t * i, o = e * n, a = new Uint16Array((t + 1) * (2 * o) + (e + 1) * (2 * r) + 8), s = 0; s < t + 1; ++s) {
    for (var u = 0; u < o; ++u) {
      var l = s * i;
      a[2 * (s * o + u) + 0] = u * (r + 1) + l, a[2 * (s * o + u) + 1] = (u + 1) * (r + 1) + l;
    }
  }

  for (var h = (t + 1) * (2 * o), d = 0; d < e + 1; ++d) {
    for (var m = 0; m < r; ++m) {
      var f = d * n;
      a[h + 2 * (m + d * r) + 0] = f * (r + 1) + m, a[h + 2 * (m + d * r) + 1] = f * (r + 1) + m + 1;
    }
  }

  return a;
}

var c = "\nin vec3 position;\n            \nin vec3 normal;\n            \nout vec3 v_positionEC;\n            \nout vec3 v_normalEC;\n            \nvoid main()\n            \n {\n               \n    v_positionEC = (czm_modelView * vec4(position, 1.0)).xyz;       \n    v_normalEC = czm_normal * normal;                               \n    gl_Position = czm_modelViewProjection * vec4(position, 1.0);\n         \n}\n           ";
var _ = "\n            \nin vec3 v_positionEC;\n            \nin vec3 v_normalEC;\n            \nuniform vec4 color;\n\n            \nvoid main(){\n               \n        vec3 positionToEyeEC = -v_positionEC;\n\n                \n        vec3 normalEC = normalize(v_normalEC);\n           \n        #ifdef FACE_FORWARD\n                \n        normalEC = faceforward(normalEC, vec3(0.0, 0.0, 1.0), -normalEC);\n           \n        #endif\n\n                \n        czm_materialInput materialInput;\n               \n        materialInput.normalEC = normalEC;\n               \n        materialInput.positionToEyeEC = positionToEyeEC;\n               \n        czm_material material = czm_getDefaultMaterial(materialInput);\n                \n        material.diffuse = color.rgb;\n               \n        material.alpha = color.a;\n\n        \n        out_FragColor = vec4(material.diffuse + material.emission, material.alpha);\n           \n        // #ifdef FLAT\n                \n        //     gl_FragColor = vec4(material.diffuse + material.emission, material.alpha);\n            \n        // #else\n               \n        //     // gl_FragColor = czm_phong(normalize(positionToEyeEC), material);\n           \n        // #endif\n           \n    }\n            ";

var CamberRadarPrimitive = function CamberRadarPrimitive(e) {
  this.innerFovRadiusPairs = e.innerFovRadiusPairs, this.outerFovRadiusPairs = e.outerFovRadiusPairs, this.radius = e.radius, this.startRadius = e.startRadius, this.modelMatrix = Cesium$b.defaultValue(e.modelMatrix, Cesium$b.Matrix4.IDENTITY), this.startFovH = Cesium$b.defaultValue(e.startFovH, Cesium$b.Math.toRadians(-50)), this.endFovH = Cesium$b.defaultValue(e.endFovH, Cesium$b.Math.toRadians(50)), this.startFovV = Cesium$b.defaultValue(e.startFovV, Cesium$b.Math.toRadians(5)), this.endFovV = Cesium$b.defaultValue(e.endFovV, Cesium$b.Math.toRadians(85)), this.segmentH = Cesium$b.defaultValue(e.segmentH, 20), this.segmentV = Cesium$b.defaultValue(e.segmentV, 20), this.subSegmentH = Cesium$b.defaultValue(e.subSegmentH, 3), this.subSegmentV = Cesium$b.defaultValue(e.subSegmentV, 3), this.color = Cesium$b.defaultValue(e.color, new Cesium$b.Color(1, 1, 0, .5)), this.lineColor = Cesium$b.defaultValue(e.lineColor, new Cesium$b.Color(1, 0, 0)), this.show = Cesium$b.defaultValue(e.show, !0), this._modelMatrix = Cesium$b.Matrix4.clone(Cesium$b.Matrix4.IDENTITY), this._startFovH = 0, this._endFovH = 0, this._startFovV = 0, this._endFovV = 0, this._segmentH = 1, this._segmentV = 1, this._subSegmentH = 1, this._subSegmentV = 1, this._boundingSphere = new Cesium$b.BoundingSphere(), this._initBoundingSphere = void 0, this._command = void 0;
};

CamberRadarPrimitive.prototype.createOuterCurveCommand = function (t) {
  var e = this._subSegmentH * this._segmentH,
      i = this._subSegmentV * this._segmentV,
      n = s(this._startFovH, this._endFovH, this._startFovV, this._endFovV, e, i, this._outerFovRadiusPairs),
      r = s(this._startFovH, this._endFovH, this._startFovV, this._endFovV, e, i, this._outerFovRadiusPairs),
      o = l$1(e, i),
      a = h$1(this._segmentH, this._segmentV, this._subSegmentH, this._subSegmentV);
  return this.createRawCommand(t, n, r, o, a);
};

CamberRadarPrimitive.prototype.createInnerCurveCommand = function (t) {
  var e = this._subSegmentH * this._segmentH,
      i = this._subSegmentV * this._segmentV,
      n = s(this._startFovH, this._endFovH, this._startFovV, this._endFovV, e, i, this._innerFovRadiusPairs),
      r = s(this._startFovH, this._endFovH, this._startFovV, this._endFovV, e, i, this._innerFovRadiusPairs),
      o = l$1(e, i),
      a = h$1(this._segmentH, this._segmentV, this._subSegmentH, this._subSegmentV);
  return this.createRawCommand(t, n, r, o, a);
};

CamberRadarPrimitive.prototype.createLeftCrossSectionCommand = function (t) {
  var e = this._subSegmentV * this._segmentV,
      i = u(this._startFovH, this._startFovV, this._endFovV, 10, e, this._innerFovRadiusPairs, this._outerFovRadiusPairs),
      n = u(this._startFovH, this._startFovV, this._endFovV, 10, e, this._innerFovRadiusPairs, this._outerFovRadiusPairs),
      r = l$1(10, e),
      o = h$1(10, this._segmentV, 1, this._subSegmentV);
  return this.createRawCommand(t, i, n, r, o);
};

CamberRadarPrimitive.prototype.createRightCrossSectionCommand = function (t) {
  var e = this._subSegmentV * this._segmentV,
      i = u(this._endFovH, this._startFovV, this._endFovV, 10, e, this._innerFovRadiusPairs, this._outerFovRadiusPairs),
      n = u(this._endFovH, this._startFovV, this._endFovV, 10, e, this._innerFovRadiusPairs, this._outerFovRadiusPairs),
      r = l$1(10, e),
      o = h$1(10, this._segmentV, 1, this._subSegmentV);
  return this.createRawCommand(t, i, n, r, o);
};

CamberRadarPrimitive.prototype.createRawCommand = function (t, e, i, n, r) {
  var o = this,
      a = Cesium$b.Appearance.getDefaultRenderState(!0, !1, void 0),
      s = Cesium$b.RenderState.fromCache(a),
      u = new Cesium$b.ShaderSource({
    sources: [c]
  }),
      l = new Cesium$b.ShaderSource({
    sources: [_]
  }),
      h = {
    color: function color() {
      return o.color;
    }
  },
      d = {
    color: function color() {
      return o.lineColor;
    }
  },
      m = Cesium$b.ShaderProgram.fromCache({
    context: t,
    vertexShaderSource: u,
    fragmentShaderSource: l,
    attributeLocations: {
      position: 0,
      normal: 1
    }
  }),
      p = Cesium$b.Buffer.createVertexBuffer({
    context: t,
    typedArray: e,
    usage: Cesium$b.BufferUsage.STATIC_DRAW
  }),
      v = Cesium$b.Buffer.createVertexBuffer({
    context: t,
    typedArray: i,
    usage: Cesium$b.BufferUsage.STATIC_DRAW
  }),
      y = Cesium$b.Buffer.createIndexBuffer({
    context: t,
    typedArray: n,
    usage: Cesium$b.BufferUsage.STATIC_DRAW,
    indexDatatype: Cesium$b.IndexDatatype.UNSIGNED_SHORT
  }),
      C = Cesium$b.Buffer.createIndexBuffer({
    context: t,
    typedArray: r,
    usage: Cesium$b.BufferUsage.STATIC_DRAW,
    indexDatatype: Cesium$b.IndexDatatype.UNSIGNED_SHORT
  }),
      w = new Cesium$b.VertexArray({
    context: t,
    attributes: [{
      index: 0,
      vertexBuffer: p,
      componentsPerAttribute: 3,
      componentDatatype: Cesium$b.ComponentDatatype.FLOAT
    }, {
      index: 1,
      vertexBuffer: v,
      componentsPerAttribute: 3,
      componentDatatype: Cesium$b.ComponentDatatype.FLOAT
    }],
    indexBuffer: y
  }),
      x = new Cesium$b.VertexArray({
    context: t,
    attributes: [{
      index: 0,
      vertexBuffer: p,
      componentsPerAttribute: 3,
      componentDatatype: Cesium$b.ComponentDatatype.FLOAT
    }, {
      index: 1,
      vertexBuffer: v,
      componentsPerAttribute: 3,
      componentDatatype: Cesium$b.ComponentDatatype.FLOAT
    }],
    indexBuffer: C
  }),
      A = Cesium$b.BoundingSphere.fromVertices(e);
  return {
    command: new Cesium$b.DrawCommand({
      vertexArray: w,
      primitiveType: Cesium$b.PrimitiveType.TRIANGLES,
      renderState: s,
      shaderProgram: m,
      uniformMap: h,
      owner: this,
      pass: Cesium$b.Pass.TRANSLUCENT,
      modelMatrix: new Cesium$b.Matrix4(),
      boundingVolume: new Cesium$b.BoundingSphere(),
      cull: !0
    }),
    lineCommand: new Cesium$b.DrawCommand({
      vertexArray: x,
      primitiveType: Cesium$b.PrimitiveType.LINES,
      renderState: s,
      shaderProgram: m,
      uniformMap: d,
      owner: this,
      pass: Cesium$b.Pass.TRANSLUCENT,
      modelMatrix: new Cesium$b.Matrix4(),
      boundingVolume: new Cesium$b.BoundingSphere(),
      cull: !0
    }),
    initBoundingSphere: A
  };
};

CamberRadarPrimitive.prototype.update = function (t) {
  var e = this;

  if (this.show) {
    (this.innerFovRadiusPairs !== this._innerFovRadiusPairs || this.outerFovRadiusPairs !== this._outerFovRadiusPairs || this.startFovH !== this._startFovH || this.endFovH !== this._endFovH || this.startFovV !== this._startFovV || this.endFovV !== this._endFovV || this.segmentH !== this._segmentH || this.segmentV !== this._segmentV || this.subSegmentH !== this._subSegmentH || this.subSegmentV !== this._subSegmentV) && (this._innerFovRadiusPairs = this.innerFovRadiusPairs, this._outerFovRadiusPairs = this.outerFovRadiusPairs, this._startFovH = this.startFovH, this._endFovH = this.endFovH, this._startFovV = this.startFovV, this._endFovV = this.endFovV, this._segmentH = this.segmentH, this._segmentV = this.segmentV, this._subSegmentH = this.subSegmentH, this._subSegmentV = this.subSegmentV, this._modelMatrix = Cesium$b.clone(Cesium$b.Matrix4.IDENTITY), this.destroyCommands()), Cesium$b.defined(this._commands) && 0 !== this._commands.length || (this._commands || (this._commands = []), this._commands.push(this.createOuterCurveCommand(t.context)), this._commands.push(this.createLeftCrossSectionCommand(t.context)), this._commands.push(this.createRightCrossSectionCommand(t.context)), this._commands.push(this.createInnerCurveCommand(t.context))), Cesium$b.Matrix4.equals(this.modelMatrix, this._modelMatrix) || (Cesium$b.Matrix4.clone(this.modelMatrix, this._modelMatrix), this._commands.forEach(function (t) {
      t.command.modelMatrix = Cesium$b.Matrix4.IDENTITY, t.command.modelMatrix = e._modelMatrix, t.command.boundingVolume = Cesium$b.BoundingSphere.transform(t.initBoundingSphere, e._modelMatrix, e._boundingSphere), t.lineCommand.modelMatrix = Cesium$b.Matrix4.IDENTITY, t.lineCommand.modelMatrix = e._modelMatrix, t.lineCommand.boundingVolume = Cesium$b.BoundingSphere.transform(t.initBoundingSphere, e._modelMatrix, e._boundingSphere);
    })), this._commands.forEach(function (e) {
      e.command && t.commandList.push(e.command), e.lineCommand && t.commandList.push(e.lineCommand);
    });
  }
};

CamberRadarPrimitive.prototype.destroyCommands = function () {
  this._commands && this._commands.forEach(function (t) {
    Cesium$b.defined(t.command) && (t.command.shaderProgram = t.command.shaderProgram && t.command.shaderProgram.destroy(), t.command.vertexArray = t.command.vertexArray && t.command.vertexArray.destroy(), t.command = void 0), Cesium$b.defined(t.lineCommand) && (t.lineCommand.shaderProgram = t.lineCommand.shaderProgram && t.lineCommand.shaderProgram.destroy(), t.lineCommand.vertexArray = t.lineCommand.vertexArray && t.lineCommand.vertexArray.destroy(), t.lineCommand = void 0);
  }), this._commands && (this._commands.length = 0);
};

CamberRadarPrimitive.prototype.destroy = function () {
  return this.destroyCommands(), Cesium$b.destroyObject(this);
};

Object.defineProperties(CamberRadarPrimitive.prototype, {
  startRadius: {
    get: function get() {
      return this._startRadius;
    },
    set: function set(t) {
      this._startRadius = t, this.innerFovRadiusPairs = [{
        fov: Cesium$b.Math.toRadians(0),
        radius: t
      }, {
        fov: Cesium$b.Math.toRadians(10),
        radius: .9 * t
      }, {
        fov: Cesium$b.Math.toRadians(20),
        radius: .8 * t
      }, {
        fov: Cesium$b.Math.toRadians(30),
        radius: .7 * t
      }, {
        fov: Cesium$b.Math.toRadians(40),
        radius: .6 * t
      }, {
        fov: Cesium$b.Math.toRadians(50),
        radius: .5 * t
      }, {
        fov: Cesium$b.Math.toRadians(60),
        radius: .4 * t
      }, {
        fov: Cesium$b.Math.toRadians(70),
        radius: .3 * t
      }, {
        fov: Cesium$b.Math.toRadians(80),
        radius: .1 * t
      }, {
        fov: Cesium$b.Math.toRadians(90),
        radius: .01 * t
      }];
    }
  },
  radius: {
    get: function get() {
      return this._radius;
    },
    set: function set(t) {
      this._radius = t, this.outerFovRadiusPairs = [{
        fov: Cesium$b.Math.toRadians(0),
        radius: t
      }, {
        fov: Cesium$b.Math.toRadians(10),
        radius: .9 * t
      }, {
        fov: Cesium$b.Math.toRadians(20),
        radius: .8 * t
      }, {
        fov: Cesium$b.Math.toRadians(30),
        radius: .7 * t
      }, {
        fov: Cesium$b.Math.toRadians(40),
        radius: .6 * t
      }, {
        fov: Cesium$b.Math.toRadians(50),
        radius: .5 * t
      }, {
        fov: Cesium$b.Math.toRadians(60),
        radius: .4 * t
      }, {
        fov: Cesium$b.Math.toRadians(70),
        radius: .3 * t
      }, {
        fov: Cesium$b.Math.toRadians(80),
        radius: .1 * t
      }, {
        fov: Cesium$b.Math.toRadians(90),
        radius: .01 * t
      }];
    }
  }
});

var Cesium$c = window.Cesium;

function getNormal(t) {
  for (var e, i, n, r, o = t.attributes.normal.values, a = 0; a < o.length; a += 3) {
    e = o[a], i = o[a + 1], n = o[a + 2], r = 1 / Math.sqrt(e * e + i * i + n * n), o[a] = e * r, o[a + 1] = i * r, o[a + 2] = n * r;
  }
}

function computeVertexNormals(t) {
  var e = t.indices,
      i = t.attributes,
      n = e.length;

  if (i.position) {
    var o = i.position.values;
    if (void 0 === i.normal) i.normal = new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.FLOAT,
      componentsPerAttribute: 3,
      values: new Float32Array(o.length)
    });else for (var a = i.normal.values, s = 0; s < n; s++) {
      a[s] = 0;
    }

    for (var u, l, h, d = i.normal.values, m = new Cesium$c.Cartesian3(), f = new Cesium$c.Cartesian3(), p = new Cesium$c.Cartesian3(), c = new Cesium$c.Cartesian3(), _ = new Cesium$c.Cartesian3(), s = 0; s < n; s += 3) {
      u = 3 * e[s + 0], l = 3 * e[s + 1], h = 3 * e[s + 2], Cesium$c.Cartesian3.fromArray(o, u, m), Cesium$c.Cartesian3.fromArray(o, l, f), Cesium$c.Cartesian3.fromArray(o, h, p), Cesium$c.Cartesian3.subtract(p, f, c), Cesium$c.Cartesian3.subtract(m, f, _), Cesium$c.Cartesian3.cross(c, _, c), d[u] += c.x, d[u + 1] += c.y, d[u + 2] += c.z, d[l] += c.x, d[l + 1] += c.y, d[l + 2] += c.z, d[h] += c.x, d[h + 1] += c.y, d[h + 2] += c.z;
    }

    getNormal(t), i.normal.needsUpdate = !0;
  }

  return t;
}

function CylinderGeometry(t) {
  this.length = t.length, this.topRadius = t.topRadius, this.bottomRadius = t.bottomRadius, this.slices = t.slices ? t.slices : 64, this.zReverse = t.zReverse;
}

var s$1 = new Cesium$c.Cartesian2();
var u$1 = new Cesium$c.Cartesian3();
var l$2 = new Cesium$c.Ray();

CylinderGeometry._createGeometry = function (t) {
  var e = t.length,
      i = t.topRadius,
      n = t.bottomRadius,
      r = t.slices,
      a = 2 * Math.PI / (r - 1),
      u = t.zReverse,
      l = [],
      h = [],
      d = [],
      m = [],
      f = [n, i],
      p = [0, u ? -e : e],
      c = 0,
      _ = Math.atan2(n - i, e),
      g = s$1;

  g.z = Math.sin(_);

  for (var v = Math.cos(_), y = 0; y < p.length; y++) {
    m[y] = [];

    for (var C = f[y], w = 0; w < r; w++) {
      m[y].push(c++);
      var x = a * w,
          A = C * Math.cos(x),
          b = C * Math.sin(x);
      l.push(A, b, p[y]), A = v * Math.cos(x), b = v * Math.sin(x), h.push(A, b, g.z), d.push(y / (p.length - 1), 0);
    }
  }

  for (var M = [], y = 1; y < p.length; y++) {
    for (var w = 1; w < r; w++) {
      var P = m[y - 1][w - 1],
          S = m[y][w - 1],
          F = m[y][w],
          E = m[y - 1][w];
      M.push(F), M.push(E), M.push(P), M.push(F), M.push(P), M.push(S), w == m[y].length - 1 && (P = m[y - 1][w], S = m[y][w], F = m[y][0], E = m[y - 1][0], M.push(F), M.push(E), M.push(P), M.push(F), M.push(P), M.push(S));
    }
  }

  M = new Int16Array(M), l = new Float32Array(l), h = new Float32Array(h), d = new Float32Array(d);
  var T = {
    position: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.DOUBLE,
      componentsPerAttribute: 3,
      values: l
    }),
    normal: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.FLOAT,
      componentsPerAttribute: 3,
      values: h
    }),
    st: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.FLOAT,
      componentsPerAttribute: 2,
      values: d
    })
  },
      R = Cesium$c.BoundingSphere.fromVertices(l),
      G = new Cesium$c.Geometry({
    attributes: T,
    indices: M,
    primitiveType: Cesium$c.PrimitiveType.TRIANGLES,
    boundingSphere: R
  });
  return l = [], M = [], d = [], G;
};

CylinderGeometry.createGeometry = function (t, e) {
  if (!e) return this._createGeometry(t);
  Cesium$c.Matrix4.multiplyByPoint(e, Cesium$c.Cartesian3.ZERO, u$1), u$1.clone(l$2.origin);
  var i = t.length,
      r = t.topRadius,
      s = (t.bottomRadius, t.slices),
      h = 2 * Math.PI / (s - 1),
      d = t.zReverse,
      m = [],
      f = [],
      p = [],
      c = [],
      _ = [0, d ? -i : i],
      g = 0,
      g = 0;
  m.push(0, 0, 0), f.push(1, 1), g++;

  for (var v = new Cesium$c.Cartesian3(), y = r / 15, C = 0; C < 16; C++) {
    for (var w = y * C, x = [], A = 0; A < s; A++) {
      var b = h * A,
          M = w * Math.cos(b),
          P = w * Math.sin(b);
      v.x = M, v.y = P, v.z = _[1];
      var S = (0, a.extend2Earth)(v, e, l$2);
      S ? (x.push(g), m.push(M, P, _[1]), f.push(C / 15, 1), g++) : (S = u$1, x.push(-1));
    }

    c.push(x);
  }

  for (var F, E, T = [0, c.length - 1], R = 0; R < T.length; R++) {
    for (var C = T[R], A = 1; A < c[C].length; A++) {
      F = c[C][A - 1], E = c[C][A], F >= 0 && E >= 0 && p.push(0, F, E);
    }
  }

  m = new Float32Array(m), p = new Int32Array(p), f = new Float32Array(f);
  var G = {
    position: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.DOUBLE,
      componentsPerAttribute: 3,
      values: m
    }),
    st: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.FLOAT,
      componentsPerAttribute: 2,
      values: f
    })
  },
      V = Cesium$c.BoundingSphere.fromVertices(m),
      D = new Cesium$c.Geometry({
    attributes: G,
    indices: p,
    primitiveType: Cesium$c.PrimitiveType.TRIANGLES,
    boundingSphere: V
  });
  return (computeVertexNormals)(D), m = [], p = [], D;
};

CylinderGeometry.createOutlineGeometry = function (t) {
  var e = t.length,
      i = t.topRadius,
      n = t.bottomRadius,
      r = t.slices,
      a = 2 * Math.PI / (r - 1),
      u = t.zReverse,
      l = [],
      h = [],
      d = [],
      m = [],
      f = [n, i],
      p = [0, u ? -e : e],
      c = 0,
      _ = Math.atan2(n - i, e),
      g = s$1;

  g.z = Math.sin(_);

  for (var v = Math.cos(_), y = 0; y < p.length; y++) {
    m[y] = [];

    for (var C = f[y], w = 0; w < r; w++) {
      m[y].push(c++);
      var x = a * w,
          A = C * Math.cos(x),
          b = C * Math.sin(x);
      l.push(A, b, p[y]), A = v * Math.cos(x), b = v * Math.sin(x), h.push(A, b, g.z), d.push(y / (p.length - 1), 0);
    }
  }

  for (var M = [], y = 1; y < p.length; y++) {
    for (var w = 1; w < r; w += 1) {
      var P = m[y - 1][w - 1],
          S = m[y][w - 1];
      m[y][w], m[y - 1][w];
      w % 8 == 1 && M.push(P, S);
    }
  }

  M = new Int16Array(M), l = new Float32Array(l), h = new Float32Array(h), d = new Float32Array(d);
  var F = {
    position: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.DOUBLE,
      componentsPerAttribute: 3,
      values: l
    }),
    normal: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.FLOAT,
      componentsPerAttribute: 3,
      values: h
    }),
    st: new Cesium$c.GeometryAttribute({
      componentDatatype: Cesium$c.ComponentDatatype.FLOAT,
      componentsPerAttribute: 2,
      values: d
    })
  },
      E = Cesium$c.BoundingSphere.fromVertices(l),
      T = new Cesium$c.Geometry({
    attributes: F,
    indices: M,
    primitiveType: Cesium$c.PrimitiveType.LINES,
    boundingSphere: E
  });
  return l = [], M = [], d = [], T;
};

CylinderGeometry.fromAngleAndLength = function (t, e, i) {
  return t = Cesium$c.Math.toRadians(t), new n({
    topRadius: Math.tan(t) * e / 2,
    bottomRadius: 0,
    length: e,
    zReverse: i
  });
};

var Cesium$d = window.Cesium;

var CylinderRadarPrimitive = /*#__PURE__*/function () {
  function CylinderRadarPrimitive(viewer, opt) {
    _classCallCheck(this, CylinderRadarPrimitive);

    this.viewer = viewer;
    this.opt = opt || {};
    this._geometry = undefined;
    if (this.opt.angle > 90) this.opt.angle = 89.9;
    if (this.opt.angle < 0) this.opt.angle = 0.1; // 张角

    this._angle = 90 - this.opt.angle; // 半径

    this._radius = this.opt.radius ? this.opt.radius : 5; // 中心点坐标；

    this._position = this.opt.position; // 姿态

    this._rotation = this.opt.rotation ? this.opt.rotation : {
      heading: 0,
      pitch: 0,
      roll: 0
    }; // 颜色

    this._color = this.opt.color ? this.opt.color : Cesium$d.Color.YELLOW; // 边框线颜色

    this._lineColor = this.opt.lineColor ? this.opt.lineColor : Cesium$d.Color.WHITE; // 是否展示

    this._visible = Cesium$d.defaultValue(this.opt.visible, true);
    this._outline = Cesium$d.defaultValue(this.opt.outline, true);
    this._topVisible = Cesium$d.defaultValue(this.opt.topVisible, true);
    this._topOutline = Cesium$d.defaultValue(this.opt.topOutline, true);
    this._modelMatrix = Cesium$d.Matrix4.clone(Cesium$d.Matrix4.IDENTITY);
    this._quaternion = new Cesium$d.Quaternion();
    this._translation = new Cesium$d.Cartesian3();
    this._scale = new Cesium$d.Cartesian3(1, 1, 1);
    this._matrix = new Cesium$d.Matrix4();
    this._inverseMatrix = new Cesium$d.Matrix4();
    this._positionCartographic = new Cesium$d.Cartographic();
    this._positionCartesian = null;
    this._drawCommands = [];
    this.updateGeometry();
  }

  _createClass(CylinderRadarPrimitive, [{
    key: "color",
    get: function get() {
      return this._color;
    },
    set: function set(color) {
      this._color = color;
    }
  }, {
    key: "lineColor",
    get: function get() {
      return this._lineColor;
    },
    set: function set(color) {
      this._lineColor = color;
    }
  }, {
    key: "position",
    get: function get() {
      return this._position;
    },
    set: function set(position) {
      this._position = position.clone();
    }
  }, {
    key: "visible",
    get: function get() {
      return this._visible;
    },
    set: function set(visible) {
      this._visible = visible;
    }
  }, {
    key: "topVisible",
    get: function get() {
      return this._topVisible;
    },
    set: function set(visible) {
      this._topVisible = visible;
    }
  }, {
    key: "angle",
    get: function get() {
      return this._angle;
    },
    set: function set(angle) {
      this._angle = 90 - angle <= 0 ? 0.1 : 90 - angle;
    }
  }, {
    key: "topOutline",
    get: function get() {
      return this._topOutline;
    },
    set: function set(topOutline) {
      this._topOutline = topOutline;
    }
  }, {
    key: "radius",
    get: function get() {
      return this._radius;
    },
    set: function set(radius) {
      return this._radius = radius;
    }
  }, {
    key: "computeMatrix",
    value: function computeMatrix(t, e) {
      if (this._positionCartesian || (this._positionCartesian = new Cesium$d.Cartesian3()), this.position instanceof Cesium$d.Cartesian3 ? this._positionCartesian = this.position : "function" == typeof this.position.getValue ? this._positionCartesian = this.position.getValue(t) : this.position._value && this.position._value instanceof Cesium$d.Cartesian3 && (this._positionCartesian = this.position._value), this._trackedEntity && this._trackedEntity.position) {
        var i = this._positionCartesian,
            n = Cesium$d.Property.getValueOrUndefined(this._trackedEntity.position, t, l);

        if (n) {
          this._trackedEntityPosition = n;
          var o = radarSpace.matrix.getHeadingPitchRollForLine(i, n, this.viewer.scene.globe.ellipsoid);
          this._rotation.heading = o.heading, this._rotation.pitch = o.pitch, this._rotation.roll = o.roll;
        }
      }

      return this._modelMatrix = Cesium$d.Transforms.eastNorthUpToFixedFrame(this._positionCartesian, this.viewer.scene.globe.ellipsoid, this._modelMatrix), this._positionCartographic = Cesium$d.Cartographic.fromCartesian(this._positionCartesian, this.viewer.scene.globe.ellipsoid, this._positionCartographic), Cesium$d.Transforms.eastNorthUpToFixedFrame(this._positionCartesian, this.viewer.scene.globe.ellipsoid, this._modelMatrix), Cesium$d.Quaternion.fromHeadingPitchRoll(this._rotation, this._quaternion), this._matrix = Cesium$d.Matrix4.fromTranslationQuaternionRotationScale(this._translation, this._quaternion, this._scale, this._matrix), Cesium$d.Matrix4.multiplyTransformation(this._modelMatrix, this._matrix, this._matrix), Cesium$d.Matrix4.inverseTransformation(this._matrix, this._inverseMatrix), this._matrix;
    }
  }, {
    key: "getTopGeometry",
    value: function getTopGeometry() {
      for (var t = this.radius, e = [], i = [], n = [], r = [], o = 90 - parseInt(this.angle), s = o < 1 ? o / 8 : 1, l = 2 * Math.PI / 127, h = 0, d = this.angle; d < 91; d += s) {
        var m = Cesium$d.Math.toRadians(d < 90 ? d : 90);
        m = Math.cos(m) * t;

        for (var f = [], p = 0; p < 128; p++) {
          var c = l * p,
              _ = m * Math.cos(c),
              g = m * Math.sin(c),
              v = Math.sqrt(t * t - _ * _ - g * g);

          e.push(_, g, v), i.push(1, 1), f.push(h++);
        }

        r.push(f);
      }

      for (var d = 1; d < r.length; d++) {
        for (var p = 1; p < r[d].length; p++) {
          var y = r[d - 1][p - 1],
              C = r[d][p - 1],
              w = r[d][p],
              x = r[d - 1][p];
          n.push(y, C, w), n.push(y, w, x);
        }
      }

      e = new Float32Array(e), n = new Int32Array(n), i = new Float32Array(i);
      var A = {
        position: new Cesium$d.GeometryAttribute({
          componentDatatype: Cesium$d.ComponentDatatype.DOUBLE,
          componentsPerAttribute: 3,
          values: e
        }),
        st: new Cesium$d.GeometryAttribute({
          componentDatatype: Cesium$d.ComponentDatatype.FLOAT,
          componentsPerAttribute: 2,
          values: i
        })
      },
          b = Cesium$d.BoundingSphere.fromVertices(e),
          M = new Cesium$d.Geometry({
        attributes: A,
        indices: n,
        primitiveType: Cesium$d.PrimitiveType.TRIANGLES,
        boundingSphere: b
      });
      return (0, this.computeVertexNormals)(M), M;
    }
  }, {
    key: "getTopOutlineGeometry",
    value: function getTopOutlineGeometry() {
      for (var t = this.radius, e = [], i = [], n = [], r = [], o = 90 - parseInt(this.angle), s = o < 1 ? o / 8 : 1, l = 2 * Math.PI / 127, h = 0, d = this.angle; d < 91; d += s) {
        var m = Cesium$d.Math.toRadians(d < 90 ? d : 90);
        m = Math.cos(m) * t;

        for (var f = [], p = 0; p < 128; p++) {
          var c = l * p,
              _ = m * Math.cos(c),
              g = m * Math.sin(c),
              v = Math.sqrt(t * t - _ * _ - g * g);

          e.push(_, g, v), i.push(1, 1), f.push(h++);
        }

        r.push(f);
      }

      for (var d = 1; d < r.length; d++) {
        for (var p = 1; p < r[d].length; p++) {
          var y = r[d - 1][p - 1],
              C = r[d][p - 1],
              w = r[d][p];
          r[d - 1][p];
          p % 8 == 1 && n.push(y, C), d % 8 == 1 && n.push(C, w);
        }
      }

      e = new Float32Array(e), n = new Int32Array(n), i = new Float32Array(i);
      var x = {
        position: new Cesium$d.GeometryAttribute({
          componentDatatype: Cesium$d.ComponentDatatype.DOUBLE,
          componentsPerAttribute: 3,
          values: e
        }),
        st: new Cesium$d.GeometryAttribute({
          componentDatatype: Cesium$d.ComponentDatatype.FLOAT,
          componentsPerAttribute: 2,
          values: i
        })
      },
          A = Cesium$d.BoundingSphere.fromVertices(e),
          b = new Cesium$d.Geometry({
        attributes: x,
        indices: n,
        primitiveType: Cesium$d.PrimitiveType.LINES,
        boundingSphere: A
      });
      return (0, this.computeVertexNormals)(b), b;
    }
  }, {
    key: "updateGeometry",
    value: function updateGeometry() {
      this._geometry = CylinderGeometry.createGeometry(new CylinderGeometry({
        topRadius: this._radius * Math.cos(Cesium$d.Math.toRadians(this.angle)),
        bottomRadius: 0,
        length: this._radius * Math.sin(Cesium$d.Math.toRadians(this.angle))
      }));
      this._topGeometry = this.getTopGeometry(), this._topOutlineGeometry = this.getTopOutlineGeometry(), this._outlineGeometry = CylinderGeometry.createOutlineGeometry(new CylinderGeometry({
        topRadius: this._radius * Math.cos(Cesium$d.Math.toRadians(this.angle)),
        bottomRadius: 0,
        slices: 128,
        length: this._radius * Math.sin(Cesium$d.Math.toRadians(this.angle))
      }));
      this._positions = new Float32Array(this._geometry.attributes.position.values.length);

      for (var t = 0; t < this._positions.length; t++) {
        this._positions[t] = this._geometry.attributes.position.values[t];
      }

      this._drawCommands && this._drawCommands.length && (this._drawCommands.forEach(function (t) {
        t.vertexArray = t.vertexArray && t.vertexArray.destroy();
      }), this._drawCommands.splice(0, this._drawCommands.length));
    }
  }, {
    key: "update",
    value: function update(t) {
      if (this._visible) {
        this.updateGeometry();
        this.computeMatrix(t.time);
        this._geometry.boundingSphere = Cesium$d.BoundingSphere.fromVertices(this._geometry.attributes.position.values);

        if (this._drawCommands.length == 0) {
          this._drawCommands.push(this.createDrawCommand(this._geometry, t));

          if (this._outline) {
            this._drawCommands.push(this.createDrawCommand(this._outlineGeometry, t));
          }

          if (this._topVisible) {
            this._drawCommands.push(this.createDrawCommand(this._topGeometry, t));

            if (this._topOutline) this._drawCommands.push(this.createDrawCommand(this._topOutlineGeometry, t));
          }
        }

        this._drawCommands.forEach(function (e) {
          t.commandList.push(e);
        });
      }
    }
  }, {
    key: "destroyCommands",
    value: function destroyCommands() {
      this._drawCommands && this._drawCommands.forEach(function (t) {
        Cesium$d.defined(t.command) && (t.command.shaderProgram = t.command.shaderProgram && t.command.shaderProgram.destroy(), t.command.vertexArray = t.command.vertexArray && t.command.vertexArray.destroy(), t.command = void 0);
      }), this._drawCommands && (this._drawCommands.length = 0);
    }
  }, {
    key: "getFragmentShaderSource",
    value: function getFragmentShaderSource(t) {
      return "\n\n        in vec3 v_position;\n\n        in vec3 v_normal;\n\n        uniform float picked;\n\n        uniform vec4  pickedColor;\n\n        uniform vec4  defaultColor;\n\n        uniform float specular;\n\n        uniform float shininess;\n\n        uniform vec3  emission;\n\n        in vec2 v_st;\n\n        uniform bool isLine;\n\n        uniform float glowPower;\n\n        void main() {\n    vec3 positionToEyeEC = -v_position; \n    vec3 normalEC =normalize(v_normal);\n    vec4 color=defaultColor;\n    if(picked!=0.0){\n        color = pickedColor;\n    }\n    //if(v_st.x<0.5){\n    //    color.a =0.75-v_st.x; \n    //}\n    //else  {\n    //    color.a =v_st.x-0.25; \n    //}\n    czm_material material;\n    material.specular = specular;\n    material.shininess = shininess;\n    material.normal =  normalEC;\n    material.emission =emission;//vec3(0.2,0.2,0.2);\n    material.diffuse = color.rgb ;\n    if(isLine){\n        material.alpha = 1.0; \n    }\n    else{\n        material.alpha =  color.a; \n    }\n        //float glow = glowPower / abs(v_st.t  ) - (glowPower / 0.5); \n        // \n        //material.emission = max(vec3(glow - 1.0 + color.rgb), color.rgb); \n        //if(isLine)\n        //    material.alpha = clamp(0.0, 1.0, glow) * color.a; \n         \n    \n        if(v_st.x==0.0){ \n          \n            out_FragColor = color ;\n    \n        }else { \n        \n            out_FragColor =color ;\n \n            // gl_FragColor = czm_phong(normalize(positionToEyeEC), material) ; \n    \n        } \n        \n}";
    }
  }, {
    key: "getVertexShaderSource",
    value: function getVertexShaderSource(t) {
      return "\n#ifdef GL_ES\n    precision highp float;\n\n        #endif\n\n\n        in vec3 position;\n\n        in vec2 st;\n\n        in vec3 normal;\n\n        uniform mat4 modelViewMatrix;\n\n        uniform mat3 normalMatrix;\n\n        uniform mat4 projectionMatrix;\n\n        out vec3 v_position;\n\n        out vec3 v_normal;\n\n        out vec2 v_st;\n\n\n        out vec3 v_light0Direction;\n\n\n        void main(void) \n{\n    vec4 pos =  modelViewMatrix * vec4( position,1.0);\n    v_normal =  normalMatrix *  normal;\n    v_st = st;\n    v_position = pos.xyz;\n    v_light0Direction = mat3( modelViewMatrix) * vec3(1.0,1.0,1.0);\n    gl_Position =  projectionMatrix * pos;\n}";
    }
  }, {
    key: "createDrawCommand",
    value: function createDrawCommand(t, e, i) {
      var that = this;
      var n = e.context,
          r = new Cesium$d.Cartesian3();
      Cesium$d.Matrix4.multiplyByPoint(this._matrix, t.boundingSphere.center, r);
      var o = new Cesium$d.BoundingSphere(r, t.boundingSphere.radius),
          s = new Cesium$d.DrawCommand({
        modelMatrix: i || this._matrix,
        owner: this,
        primitiveType: t.primitiveType,
        pass: Cesium$d.Pass.TRANSLUCENT,
        boundingVolume: o
      }),
          u = this,
          l = Cesium$d.GeometryPipeline.createAttributeLocations(t);
      return s.vertexArray = Cesium$d.VertexArray.fromGeometry({
        context: n,
        geometry: t,
        attributeLocations: l,
        bufferUsage: Cesium$d.BufferUsage.STATIC_DRAW
      }), s.vertexArray._attributeLocations = l, s.shaderProgram = Cesium$d.ShaderProgram.replaceCache({
        context: n,
        vertexShaderSource: this.getVertexShaderSource(t),
        fragmentShaderSource: this.getFragmentShaderSource(t),
        attributeLocations: l
      }), s.renderState = Cesium$d.RenderState.fromCache({
        blending: Cesium$d.BlendingState.ALPHA_BLEND,
        depthTest: {
          enabled: true,
          func: Cesium$d.DepthFunction.LESS
        },
        cull: {
          enabled: false,
          face: Cesium$d.CullFace.BACK
        }
      }), s.uniformMap = {}, s.uniformMap.projectionMatrix = function () {
        return e.context.uniformState.projection;
      }, s.uniformMap.modelViewMatrix = function () {
        return e.context.uniformState.modelView;
      }, s.uniformMap.shininess = function () {
        return u.shininess || (u.shininess = 0), u.shininess;
      }, s.uniformMap.emission = function () {
        return u.emission || (u.emission = new Cesium$d.Cartesian3(.2, .2, .2)), u.emission;
      }, s.uniformMap.specular = function () {
        return u.specular || (u.specular = 0), u.specular;
      }, s.uniformMap.isLine = function () {
        return t.primitiveType == Cesium$d.PrimitiveType.LINES || t.primitiveType == Cesium$d.PrimitiveType.LINE_STRIP;
      }, s.uniformMap.defaultColor = function () {
        var color = Cesium$d.Color.BLUE;

        if (t.primitiveType == Cesium$d.PrimitiveType.LINES) {
          color = that._lineColor;
        } else {
          color = that._color;
        }

        return color;
      }, s.uniformMap.picked = function () {
        return u.picked || (u.picked = 0), u.picked;
      }, s.uniformMap.pickedColor = function () {
        return u.pickedColor || (u.pickedColor = new Cesium$d.Color(1, 1, 0, 1)), u.pickedColor;
      }, s.uniformMap.normalMatrix = function () {
        return e.context.uniformState.normal;
      }, s.uniformMap.glowPower = function () {
        return .25;
      }, s;
    }
  }, {
    key: "destroy",
    value: function destroy(t) {
      t && (this.viewer.scene.primitives.remove(this), this._drawCommands.forEach(function (t) {
        t.vertexArray = t.vertexArray && t.vertexArray.destroy();
      }), this._drawCommands = []);
    }
  }, {
    key: "getNormal",
    value: function getNormal(t) {
      for (var e, i, n, r, o = t.attributes.normal.values, a = 0; a < o.length; a += 3) {
        e = o[a], i = o[a + 1], n = o[a + 2], r = 1 / Math.sqrt(e * e + i * i + n * n), o[a] = e * r, o[a + 1] = i * r, o[a + 2] = n * r;
      }
    }
  }, {
    key: "computeVertexNormals",
    value: function computeVertexNormals(t) {
      var e = t.indices,
          i = t.attributes,
          n = e.length;

      if (i.position) {
        var o = i.position.values;
        if (void 0 === i.normal) i.normal = new Cesium$d.GeometryAttribute({
          componentDatatype: Cesium$d.ComponentDatatype.FLOAT,
          componentsPerAttribute: 3,
          values: new Float32Array(o.length)
        });else for (var a = i.normal.values, s = 0; s < n; s++) {
          a[s] = 0;
        }

        for (var u, l, h, d = i.normal.values, m = new Cesium$d.Cartesian3(), f = new Cesium$d.Cartesian3(), p = new Cesium$d.Cartesian3(), c = new Cesium$d.Cartesian3(), _ = new Cesium$d.Cartesian3(), s = 0; s < n; s += 3) {
          u = 3 * e[s + 0], l = 3 * e[s + 1], h = 3 * e[s + 2], Cesium$d.Cartesian3.fromArray(o, u, m), Cesium$d.Cartesian3.fromArray(o, l, f), Cesium$d.Cartesian3.fromArray(o, h, p), Cesium$d.Cartesian3.subtract(p, f, c), Cesium$d.Cartesian3.subtract(m, f, _), Cesium$d.Cartesian3.cross(c, _, c), d[u] += c.x, d[u + 1] += c.y, d[u + 2] += c.z, d[l] += c.x, d[l + 1] += c.y, d[l + 2] += c.z, d[h] += c.x, d[h + 1] += c.y, d[h + 2] += c.z;
        }

        for (var e, ii, n, r, o = t.attributes.normal.values, a = 0; a < o.length; a += 3) {
          e = o[a], ii = o[a + 1], n = o[a + 2], r = 1 / Math.sqrt(e * e + ii * ii + n * n), o[a] = e * r, o[a + 1] = ii * r, o[a + 2] = n * r;
        }

        i.normal.needsUpdate = true;
      }

      return t;
    }
  }]);

  return CylinderRadarPrimitive;
}();

/* 参考 https://github.com/kaktus40/cesium-sensors/tree/master */
var shader1 = "\n\t#ifdef GL_OES_standard_derivatives\n    \n\t#extension GL_OES_standard_derivatives : enable\n\n\t#endif\n\n\n\tuniform bool u_showIntersection;\n\n\tuniform bool u_showThroughEllipsoid;\n\n\n\tuniform float u_radius;\n\n\tuniform float u_xHalfAngle;\n\n\tuniform float u_yHalfAngle;\n\n\tuniform float u_normalDirection;\n\n\tuniform float u_type;\n\n\n\tin vec3 v_position;\n\n\tin vec3 v_positionWC;\n\n\tin vec3 v_positionEC;\n\n\tin vec3 v_normalEC;\n\n\n\tvec4 getColor(float sensorRadius, vec3 pointEC)\n\n\t{\n    czm_materialInput materialInput;\n\n    vec3 pointMC = (czm_inverseModelView * vec4(pointEC, 1.0)).xyz;\n    materialInput.st = sensor2dTextureCoordinates(sensorRadius, pointMC);\n    materialInput.str = pointMC / sensorRadius;\n\n    vec3 positionToEyeEC = -v_positionEC;\n    materialInput.positionToEyeEC = positionToEyeEC;\n\n    vec3 normalEC = normalize(v_normalEC);\n    materialInput.normalEC = u_normalDirection * normalEC;\n\n    czm_material material = czm_getMaterial(materialInput);\n    // czm_lightDirectionEC\u5728cesium1.66\u5F00\u59CB\u52A0\u5165\u7684\n    return mix(czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC), vec4(material.diffuse, material.alpha), 0.4);\n\n}\n\nbool isOnBoundary(float value, float epsilon)\n{\n    float width = getIntersectionWidth();\n    float tolerance = width * epsilon;\n\n#ifdef GL_OES_standard_derivatives\n    float delta = max(abs(dFdx(value)), abs(dFdy(value)));\n    float pixels = width * delta;\n    float temp = abs(value);\n    // There are a couple things going on here.\n    // First we test the value at the current fragment to see if it is within the tolerance.\n    // We also want to check if the value of an adjacent pixel is within the tolerance,\n    // but we don't want to admit points that are obviously not on the surface.\n    // For example, if we are looking for \"value\" to be close to 0, but value is 1 and the adjacent value is 2,\n    // then the delta would be 1 and \"temp - delta\" would be \"1 - 1\" which is zero even though neither of\n    // the points is close to zero.\n    return temp < tolerance && temp < pixels || (delta < 10.0 * tolerance && temp - delta < tolerance && temp < pixels);\n#else\n    return abs(value) < tolerance;\n#endif\n}\n\nvec4 shade(bool isOnBoundary)\n{\n    if (u_showIntersection && isOnBoundary)\n    {\n        return getIntersectionColor();\n    }\n    if(u_type == 1.0){\n        return getLineColor();\n    }\n    return getColor(u_radius, v_positionEC);\n}\n\nfloat ellipsoidSurfaceFunction(vec3 point)\n{\n    vec3 scaled = czm_ellipsoidInverseRadii * point;\n    return dot(scaled, scaled) - 1.0;\n}\n\nvoid main()\n{\n    vec3 sensorVertexWC = czm_model[3].xyz;      // (0.0, 0.0, 0.0) in model coordinates\n    vec3 sensorVertexEC = czm_modelView[3].xyz;  // (0.0, 0.0, 0.0) in model coordinates\n\n    //vec3 pixDir = normalize(v_position);\n    float positionX = v_position.x;\n    float positionY = v_position.y;\n    float positionZ = v_position.z;\n\n    vec3 zDir = vec3(0.0, 0.0, 1.0);\n    vec3 lineX = vec3(positionX, 0 ,positionZ);\n    vec3 lineY = vec3(0, positionY, positionZ);\n    float resX = dot(normalize(lineX), zDir);\n    if(resX < cos(u_xHalfAngle)-0.00001){\n        discard;\n    }\n    float resY = dot(normalize(lineY), zDir);\n    if(resY < cos(u_yHalfAngle)-0.00001){\n        discard;\n    }\n\n\n    float ellipsoidValue = ellipsoidSurfaceFunction(v_positionWC);\n\n    // Occluded by the ellipsoid?\n\tif (!u_showThroughEllipsoid)\n\t{\n\t    // Discard if in the ellipsoid\n\t    // PERFORMANCE_IDEA: A coarse check for ellipsoid intersection could be done on the CPU first.\n\t    if (ellipsoidValue < 0.0)\n\t    {\n            discard;\n\t    }\n\n\t    // Discard if in the sensor's shadow\n\t    if (inSensorShadow(sensorVertexWC, v_positionWC))\n\t    {\n\t        discard;\n\t    }\n    }\n\n    // Notes: Each surface functions should have an associated tolerance based on the floating point error.\n    bool isOnEllipsoid = isOnBoundary(ellipsoidValue, czm_epsilon3);\n    //isOnEllipsoid = false;\n    //if((resX >= 0.8 && resX <= 0.81)||(resY >= 0.8 && resY <= 0.81)){\n    /*if(false){\n        out_FragColor = vec4(1.0,0.0,0.0,1.0);\n    }else{\n        out_FragColor = shade(isOnEllipsoid);\n    }\n*/\n    out_FragColor = shade(isOnEllipsoid);\n\n}\n";
var shader2 = "\n\tin vec4 position;\n\n\tin vec3 normal;\n\n\n\tout vec3 v_position;\n\n\tout vec3 v_positionWC;\n\n\tout vec3 v_positionEC;\n\n\tout vec3 v_normalEC;\n\n\n\tvoid main()\n{\n    gl_Position = czm_modelViewProjection * position;\n    v_position = vec3(position);\n    v_positionWC = (czm_model * position).xyz;\n    v_positionEC = (czm_modelView * position).xyz;\n    v_normalEC = czm_normal * normal;\n}\n\t";
var shader3 = "\nuniform vec4 u_intersectionColor;\n\nuniform float u_intersectionWidth;\n\nuniform vec4 u_lineColor;\n\n\nbool inSensorShadow(vec3 coneVertexWC, vec3 pointWC)\n{\n    // Diagonal matrix from the unscaled ellipsoid space to the scaled space.    \n    vec3 D = czm_ellipsoidInverseRadii;\n\n    // Sensor vertex in the scaled ellipsoid space\n    vec3 q = D * coneVertexWC;\n    float qMagnitudeSquared = dot(q, q);\n    float test = qMagnitudeSquared - 1.0;\n    \n    // Sensor vertex to fragment vector in the ellipsoid's scaled space\n    vec3 temp = D * pointWC - q;\n    float d = dot(temp, q);\n    \n    // Behind silhouette plane and inside silhouette cone\n    return (d < -test) && (d / length(temp) < -sqrt(test));\n}\n\n///////////////////////////////////////////////////////////////////////////////\n\nvec4 getLineColor()\n{\n    return u_lineColor;\n}\n\nvec4 getIntersectionColor()\n{\n    return u_intersectionColor;\n}\n\nfloat getIntersectionWidth()\n{\n    return u_intersectionWidth;\n}\n\nvec2 sensor2dTextureCoordinates(float sensorRadius, vec3 pointMC)\n{\n    // (s, t) both in the range [0, 1]\n    float t = pointMC.z / sensorRadius;\n    float s = 1.0 + (atan(pointMC.y, pointMC.x) / czm_twoPi);\n    s = s - floor(s);\n    \n    return vec2(s, t);\n}\n\n";
var shader4 = "\n\t#ifdef GL_OES_standard_derivatives\n\n    #extension GL_OES_standard_derivatives : enable\n#endif\n\n\n\tuniform bool u_showIntersection;\n\n\tuniform bool u_showThroughEllipsoid;\n\n\n\tuniform float u_radius;\n\n\tuniform float u_xHalfAngle;\n\n\tuniform float u_yHalfAngle;\n\n\tuniform float u_normalDirection;\n\n\tuniform vec4 u_color;\n\n\n\tin vec3 v_position;\n\n\tin vec3 v_positionWC;\n\n\tin vec3 v_positionEC;\n\n\tin vec3 v_normalEC;\n\n\n\tvec4 getColor(float sensorRadius, vec3 pointEC)\n{\n    \n\t\tczm_materialInput materialInput;\n\n    \n\t\tvec3 pointMC = (czm_inverseModelView * vec4(pointEC, 1.0)).xyz;\n    \n\t\tmaterialInput.st = sensor2dTextureCoordinates(sensorRadius, pointMC);\n    \n\t\tmaterialInput.str = pointMC / sensorRadius;\n\n    \n\t\tvec3 positionToEyeEC = -v_positionEC;\n    \n\t\tmaterialInput.positionToEyeEC = positionToEyeEC;\n\n    \n\t\tvec3 normalEC = normalize(v_normalEC);\n    \n\t\tmaterialInput.normalEC = u_normalDirection * normalEC;\n\n    \n\t\tczm_material material = czm_getMaterial(materialInput);\n\n    \n\t\tmaterial.diffuse = u_color.rgb;\n    \n\t\tmaterial.alpha = u_color.a;\n    \n\t\t// czm_lightDirectionEC\u5728cesium1.66\u5F00\u59CB\u52A0\u5165\u7684\n    \n\t\treturn mix(czm_phong(normalize(positionToEyeEC), material, czm_lightDirectionEC), vec4(material.diffuse, material.alpha), 0.4);\n\n}\n\n\n\t\tbool isOnBoundary(float value, float epsilon)\n{\n    \n\t\t\tfloat width = getIntersectionWidth();\n    \n\t\t\tfloat tolerance = width * epsilon;\n\n\n\t\t\t#ifdef GL_OES_standard_derivatives\n    \n\t\t\tfloat delta = max(abs(dFdx(value)), abs(dFdy(value)));\n    \n\t\t\tfloat pixels = width * delta;\n    \n\t\t\tfloat temp = abs(value);\n    \n\t\t\t\n\t\t\treturn temp < tolerance && temp < pixels || (delta < 10.0 * tolerance && temp - delta < tolerance && temp < pixels);\n\n\t\t\t#else\n \n\t\t\treturn abs(value) < tolerance;\n#endif\n}\n\n\n\t\t\tvec4 shade(bool isOnBoundary)\n{\n    \t\n\t\t\t\tif (u_showIntersection && isOnBoundary)\n    {\n        \n\t\t\t\t\treturn getIntersectionColor();\n    \n\t\t\t\t}\n    \n\t\t\t\treturn getColor(u_radius, v_positionEC);\n\n\t\t\t}\n\n\n\t\t\tfloat ellipsoidSurfaceFunction(vec3 point)\n{\n    \n\t\t\t\t\tvec3 scaled = czm_ellipsoidInverseRadii * point;\n    \n\t\t\t\t\treturn dot(scaled, scaled) - 1.0;\n}\n\n\n\t\t\t\t\tvoid main()\n{\n    \n\t\t\t\t\t\tvec3 sensorVertexWC = czm_model[3].xyz;      \n\t\t\t\t\t\t// (0.0, 0.0, 0.0) in model coordinates\n    \n\t\t\t\t\t\tvec3 sensorVertexEC = czm_modelView[3].xyz;  \n\t\t\t\t\t\t// (0.0, 0.0, 0.0) in model coordinates\n\n    \n\t\t\t\t\t\t//vec3 pixDir = normalize(v_position);\n    \n\t\t\t\t\t\tfloat positionX = v_position.x;\n    \n\t\t\t\t\t\tfloat positionY = v_position.y;\n    \n\t\t\t\t\t\tfloat positionZ = v_position.z;\n\n    \n\t\t\t\t\t\tvec3 zDir = vec3(0.0, 0.0, 1.0);\n    \n\t\t\t\t\t\tvec3 lineX = vec3(positionX, 0 ,positionZ);\n    \n\t\t\t\t\t\tvec3 lineY = vec3(0, positionY, positionZ);\n    \n\t\t\t\t\t\tfloat resX = dot(normalize(lineX), zDir);\n    \n\t\t\t\t\t\tif(resX < cos(u_xHalfAngle) - 0.0001){\n        \n\t\t\t\t\t\t\tdiscard;\n    \n\t\t\t\t\t\t}\n    \n\t\t\t\t\t\tfloat resY = dot(normalize(lineY), zDir);\n    \n\t\t\t\t\t\tif(resY < cos(u_yHalfAngle)- 0.0001){\n        \n\t\t\t\t\t\t\tdiscard;\n    \n\t\t\t\t\t\t}\n\n\n    \n\t\t\t\t\t\tfloat ellipsoidValue = ellipsoidSurfaceFunction(v_positionWC);\n\n    \n\t\t\t\t\t\tif (!u_showThroughEllipsoid)\n\t{\n\t    \n\t\t\t\t\t\t \n\t\t\t\t\t\tif (ellipsoidValue < 0.0)\n\t    {\n            \n\t\t\t\t\t\t\tdiscard;\n\t    }\n\n\t    \n\t\t\t\t\t\t\tif (inSensorShadow(sensorVertexWC, v_positionWC))\n\t    {\n\t        \n\t\t\t\t\t\t\t\tdiscard;\n\t    \n\t\t\t\t\t\t\t}\n    \n\t\t\t\t\t\t}\n\n    \n\t\t\t\t\t\t\n\t\t\t\t\t\tbool isOnEllipsoid = isOnBoundary(ellipsoidValue, czm_epsilon3);\n    \n\t\t\t\t\t\tout_FragColor = shade(isOnEllipsoid);\n\n\n\t\t\t}\n";
var Cesium$e = window.Cesium;

function removePrimitive(entity, hash, primitives) {
  var data = hash[entity.id];

  if (Cesium$e.defined(data)) {
    var primitive = data.primitive;
    primitives.remove(primitive);

    if (!primitive.isDestroyed()) {
      primitive.destroy();
    }

    delete hash[entity.id];
  }
}

function ScanRadarGraphics(options) {
  this._show = undefined;
  this._radius = undefined;
  this._xHalfAngle = undefined;
  this._yHalfAngle = undefined;
  this._lineColor = undefined;
  this._showSectorLines = undefined;
  this._showSectorSegmentLines = undefined;
  this._showLateralSurfaces = undefined;
  this._material = undefined;
  this._showDomeSurfaces = undefined;
  this._showDomeLines = undefined;
  this._showIntersection = undefined;
  this._intersectionColor = undefined;
  this._intersectionWidth = undefined;
  this._showThroughEllipsoid = undefined;
  this._gaze = undefined;
  this._showScanPlane = undefined;
  this._scanPlaneColor = undefined;
  this._scanPlaneMode = undefined;
  this._scanPlaneRate = undefined;
  this._definitionChanged = new Cesium$e.Event();
  this.merge(Cesium$e.defaultValue(options, Cesium$e.defaultValue.EMPTY_OBJECT));
}

Object.defineProperties(ScanRadarGraphics.prototype, {
  definitionChanged: {
    get: function get() {
      return this._definitionChanged;
    }
  },
  show: Cesium$e.createPropertyDescriptor('show'),
  radius: Cesium$e.createPropertyDescriptor('radius'),
  xHalfAngle: Cesium$e.createPropertyDescriptor('xHalfAngle'),
  yHalfAngle: Cesium$e.createPropertyDescriptor('yHalfAngle'),
  lineColor: Cesium$e.createPropertyDescriptor('lineColor'),
  showSectorLines: Cesium$e.createPropertyDescriptor('showSectorLines'),
  showSectorSegmentLines: Cesium$e.createPropertyDescriptor('showSectorSegmentLines'),
  showLateralSurfaces: Cesium$e.createPropertyDescriptor('showLateralSurfaces'),
  material: Cesium$e.createMaterialPropertyDescriptor('material'),
  showDomeSurfaces: Cesium$e.createPropertyDescriptor('showDomeSurfaces'),
  showDomeLines: Cesium$e.createPropertyDescriptor('showDomeLines '),
  showIntersection: Cesium$e.createPropertyDescriptor('showIntersection'),
  intersectionColor: Cesium$e.createPropertyDescriptor('intersectionColor'),
  intersectionWidth: Cesium$e.createPropertyDescriptor('intersectionWidth'),
  showThroughEllipsoid: Cesium$e.createPropertyDescriptor('showThroughEllipsoid'),
  gaze: Cesium$e.createPropertyDescriptor('gaze'),
  showScanPlane: Cesium$e.createPropertyDescriptor('showScanPlane'),
  scanPlaneColor: Cesium$e.createPropertyDescriptor('scanPlaneColor'),
  scanPlaneMode: Cesium$e.createPropertyDescriptor('scanPlaneMode'),
  scanPlaneRate: Cesium$e.createPropertyDescriptor('scanPlaneRate')
});

ScanRadarGraphics.prototype.clone = function (result) {
  if (!Cesium$e.defined(result)) {
    result = new ScanRadarGraphics();
  }

  result.show = this.show;
  result.radius = this.radius;
  result.xHalfAngle = this.xHalfAngle;
  result.yHalfAngle = this.yHalfAngle;
  result.lineColor = this.lineColor;
  result.showSectorLines = this.showSectorLines;
  result.showSectorSegmentLines = this.showSectorSegmentLines;
  result.showLateralSurfaces = this.showLateralSurfaces;
  result.material = this.material;
  result.showDomeSurfaces = this.showDomeSurfaces;
  result.showDomeLines = this.showDomeLines;
  result.showIntersection = this.showIntersection;
  result.intersectionColor = this.intersectionColor;
  result.intersectionWidth = this.intersectionWidth;
  result.showThroughEllipsoid = this.showThroughEllipsoid;
  result.gaze = this.gaze;
  result.showScanPlane = this.showScanPlane;
  result.scanPlaneColor = this.scanPlaneColor;
  result.scanPlaneMode = this.scanPlaneMode;
  result.scanPlaneRate = this.scanPlaneRate;
  return result;
};

ScanRadarGraphics.prototype.merge = function (source) {
  if (!Cesium$e.defined(source)) {
    throw new Cesium$e.DeveloperError('source is required.');
  }

  this.slice = Cesium$e.defaultValue(this.slice, source.slice);
  this.show = Cesium$e.defaultValue(this.show, source.show);
  this.radius = Cesium$e.defaultValue(this.radius, source.radius);
  this.xHalfAngle = Cesium$e.defaultValue(this.xHalfAngle, source.xHalfAngle);
  this.yHalfAngle = Cesium$e.defaultValue(this.yHalfAngle, source.yHalfAngle);
  this.lineColor = Cesium$e.defaultValue(this.lineColor, source.lineColor);
  this.showSectorLines = Cesium$e.defaultValue(this.showSectorLines, source.showSectorLines);
  this.showSectorSegmentLines = Cesium$e.defaultValue(this.showSectorSegmentLines, source.showSectorSegmentLines);
  this.showLateralSurfaces = Cesium$e.defaultValue(this.showLateralSurfaces, source.showLateralSurfaces);
  this.material = Cesium$e.defaultValue(this.material, source.material);
  this.showDomeSurfaces = Cesium$e.defaultValue(this.showDomeSurfaces, source.showDomeSurfaces);
  this.showDomeLines = Cesium$e.defaultValue(this.showDomeLines, source.showDomeLines);
  this.showIntersection = Cesium$e.defaultValue(this.showIntersection, source.showIntersection);
  this.intersectionColor = Cesium$e.defaultValue(this.intersectionColor, source.intersectionColor);
  this.intersectionWidth = Cesium$e.defaultValue(this.intersectionWidth, source.intersectionWidth);
  this.showThroughEllipsoid = Cesium$e.defaultValue(this.showThroughEllipsoid, source.showThroughEllipsoid);
  this.gaze = Cesium$e.defaultValue(this.gaze, source.gaze);
  this.showScanPlane = Cesium$e.defaultValue(this.showScanPlane, source.showScanPlane);
  this.scanPlaneColor = Cesium$e.defaultValue(this.scanPlaneColor, source.scanPlaneColor);
  this.scanPlaneMode = Cesium$e.defaultValue(this.scanPlaneMode, source.scanPlaneMode);
  this.scanPlaneRate = Cesium$e.defaultValue(this.scanPlaneRate, source.scanPlaneRate);
}; //====================================================================================================================


var AssociativeArray = Cesium$e.AssociativeArray;
var Cartesian3$3 = Cesium$e.Cartesian3;
var Color = Cesium$e.Color;
var defined$7 = Cesium$e.defined;
var destroyObject = Cesium$e.destroyObject;
var DeveloperError$3 = Cesium$e.DeveloperError;
var Matrix3 = Cesium$e.Matrix3;
var Matrix4$1 = Cesium$e.Matrix4;
var Quaternion = Cesium$e.Quaternion;
var MaterialProperty = Cesium$e.MaterialProperty;
var Property$1 = Cesium$e.Property;
var matrix3Scratch = new Matrix3(); // var matrix4Scratch = new Matrix4();

var cachedPosition = new Cartesian3$3();
var cachedGazePosition = new Cartesian3$3();
var cachedOrientation = new Quaternion();
var diffVectorScratch = new Cartesian3$3();
var orientationScratch = new Quaternion();

var ScanRadarVisualizer = function ScanRadarVisualizer(scene, entityCollection) {
  // >>includeStart('debug', pragmas.debug);
  if (!defined$7(scene)) {
    throw new DeveloperError$3('scene is required.');
  }

  if (!defined$7(entityCollection)) {
    throw new DeveloperError$3('entityCollection is required.');
  } // >>includeEnd('debug');


  entityCollection.collectionChanged.addEventListener(ScanRadarVisualizer.prototype._onCollectionChanged, this);
  this._scene = scene;
  this._primitives = scene.primitives;
  this._entityCollection = entityCollection;
  this._hash = {};
  this._entitiesToVisualize = new AssociativeArray();

  this._onCollectionChanged(entityCollection, entityCollection.values, [], []);
};
/**
 * Updates the primitives created by this visualizer to match their
 * Entity counterpart at the given time.
 *
 * @param {JulianDate} time The time to update to.
 * @returns {Boolean} This function always returns true.
 */


ScanRadarVisualizer.prototype.update = function (time) {
  // >>includeStart('debug', pragmas.debug);
  if (!defined$7(time)) {
    throw new DeveloperError$3('time is required.');
  } // >>includeEnd('debug');


  var entities = this._entitiesToVisualize.values;
  var hash = this._hash;
  var primitives = this._primitives;

  for (var i = 0, len = entities.length; i < len; i++) {
    var entity = entities[i];
    var rectangularSensorGraphics = entity._rectangularSensor;
    var position;
    var orientation;
    var radius;
    var xHalfAngle;
    var yHalfAngle;
    var data = hash[entity.id];
    var show = entity.isShowing && entity.isAvailable(time) && Property$1.getValueOrDefault(rectangularSensorGraphics._show, time, true);

    if (show) {
      position = Property$1.getValueOrUndefined(entity._position, time, cachedPosition);
      orientation = Property$1.getValueOrUndefined(entity._orientation, time, cachedOrientation);
      radius = Property$1.getValueOrUndefined(rectangularSensorGraphics._radius, time);
      xHalfAngle = Property$1.getValueOrUndefined(rectangularSensorGraphics._xHalfAngle, time);
      yHalfAngle = Property$1.getValueOrUndefined(rectangularSensorGraphics._yHalfAngle, time);
      show = defined$7(position) && defined$7(xHalfAngle) && defined$7(yHalfAngle);
    }

    if (!show) {
      // don't bother creating or updating anything else
      if (defined$7(data)) {
        data.primitive.show = false;
      }

      continue;
    }

    var primitive = defined$7(data) ? data.primitive : undefined;

    if (!defined$7(primitive)) {
      primitive = new ScanRadarPrimitive();
      primitive.id = entity;
      primitives.add(primitive);
      data = {
        primitive: primitive,
        position: undefined,
        orientation: undefined
      };
      hash[entity.id] = data;
    }

    var gaze = Property$1.getValueOrUndefined(rectangularSensorGraphics._gaze, time);

    if (defined$7(gaze)) {
      var targetPosition = Property$1.getValueOrUndefined(gaze._position, time, cachedGazePosition);

      if (!defined$7(position) || !defined$7(targetPosition)) {
        continue;
      }

      var diffVector = Cartesian3$3.subtract(position, targetPosition, diffVectorScratch);
      var rotate = Cartesian3$3.angleBetween(Cesium$e.Cartesian3.UNIT_Z, diffVector);
      var cross = Cartesian3$3.cross(Cesium$e.Cartesian3.UNIT_Z, diffVector, diffVectorScratch);
      var orientation = Quaternion.fromAxisAngle(cross, rotate - Math.PI, orientationScratch); //replace original radius

      radius = Cartesian3$3.distance(position, targetPosition);
      primitive.modelMatrix = Matrix4$1.fromRotationTranslation(Matrix3.fromQuaternion(orientation, matrix3Scratch), position, primitive.modelMatrix);
    } else {
      if (!Cartesian3$3.equals(position, data.position) || !Quaternion.equals(orientation, data.orientation)) {
        if (defined$7(orientation)) {
          primitive.modelMatrix = Matrix4$1.fromRotationTranslation(Matrix3.fromQuaternion(orientation, matrix3Scratch), position, primitive.modelMatrix);
          data.position = Cartesian3$3.clone(position, data.position);
          data.orientation = Quaternion.clone(orientation, data.orientation);
        } else {
          primitive.modelMatrix = Cesium$e.Transforms.eastNorthUpToFixedFrame(position);
          data.position = Cartesian3$3.clone(position, data.position);
        }
      }
    }

    primitive.show = true;
    primitive.gaze = gaze;
    primitive.radius = radius;
    primitive.xHalfAngle = xHalfAngle;
    primitive.yHalfAngle = yHalfAngle;
    primitive.lineColor = Property$1.getValueOrDefault(rectangularSensorGraphics._lineColor, time, Color.WHITE);
    primitive.showSectorLines = Property$1.getValueOrDefault(rectangularSensorGraphics._showSectorLines, time, true);
    primitive.showSectorSegmentLines = Property$1.getValueOrDefault(rectangularSensorGraphics._showSectorSegmentLines, time, true);
    primitive.showLateralSurfaces = Property$1.getValueOrDefault(rectangularSensorGraphics._showLateralSurfaces, time, true);
    primitive.material = MaterialProperty.getValue(time, rectangularSensorGraphics._material, primitive.material);
    primitive.showDomeSurfaces = Property$1.getValueOrDefault(rectangularSensorGraphics._showDomeSurfaces, time, true);
    primitive.showDomeLines = Property$1.getValueOrDefault(rectangularSensorGraphics._showDomeLines, time, true);
    primitive.showIntersection = Property$1.getValueOrDefault(rectangularSensorGraphics._showIntersection, time, true);
    primitive.intersectionColor = Property$1.getValueOrDefault(rectangularSensorGraphics._intersectionColor, time, Color.WHITE);
    primitive.intersectionWidth = Property$1.getValueOrDefault(rectangularSensorGraphics._intersectionWidth, time, 1);
    primitive.showThroughEllipsoid = Property$1.getValueOrDefault(rectangularSensorGraphics._showThroughEllipsoid, time, true);
    primitive.scanPlaneMode = Property$1.getValueOrDefault(rectangularSensorGraphics._scanPlaneMode, time);
    primitive.scanPlaneColor = Property$1.getValueOrDefault(rectangularSensorGraphics._scanPlaneColor, time, Color.WHITE);
    primitive.showScanPlane = Property$1.getValueOrDefault(rectangularSensorGraphics._showScanPlane, time, true);
    primitive.scanPlaneRate = Property$1.getValueOrDefault(rectangularSensorGraphics._scanPlaneRate, time, 1);
  }

  return true;
};
/**
 * Returns true if this object was destroyed; otherwise, false.
 *
 * @returns {Boolean} True if this object was destroyed; otherwise, false.
 */


ScanRadarVisualizer.prototype.isDestroyed = function () {
  return false;
};
/**
 * Removes and destroys all primitives created by this instance.
 */


ScanRadarVisualizer.prototype.destroy = function () {
  var entities = this._entitiesToVisualize.values;
  var hash = this._hash;
  var primitives = this._primitives;

  for (var i = entities.length - 1; i > -1; i--) {
    removePrimitive(entities[i], hash, primitives);
  }

  return destroyObject(this);
};
/**
 * @private
 */


ScanRadarVisualizer.prototype._onCollectionChanged = function (entityCollection, added, removed, changed) {
  var i;
  var entity;
  var entities = this._entitiesToVisualize;
  var hash = this._hash;
  var primitives = this._primitives;

  for (i = added.length - 1; i > -1; i--) {
    entity = added[i];

    if (defined$7(entity._rectangularSensor) && defined$7(entity._position)) {
      entities.set(entity.id, entity);
    }
  }

  for (i = changed.length - 1; i > -1; i--) {
    entity = changed[i];

    if (defined$7(entity._rectangularSensor) && defined$7(entity._position)) {
      entities.set(entity.id, entity);
    } else {
      removePrimitive(entity, hash, primitives);
      entities.remove(entity.id);
    }
  }

  for (i = removed.length - 1; i > -1; i--) {
    entity = removed[i];
    removePrimitive(entity, hash, primitives);
    entities.remove(entity.id);
  }
}; //===============================================================================================


var BoundingSphere$1 = Cesium$e.BoundingSphere;
var Cartesian3$3 = Cesium$e.Cartesian3;
var Color = Cesium$e.Color;
var combine = Cesium$e.combine;
var ComponentDatatype = Cesium$e.ComponentDatatype;
var defaultValue = Cesium$e.defaultValue;
var defined$7 = Cesium$e.defined; // var defineProperties = Cesium.defineProperties;
// var destroyObject = Cesium.destroyObject;

var DeveloperError$3 = Cesium$e.DeveloperError;
var Matrix4$1 = Cesium$e.Matrix4;
var PrimitiveType = Cesium$e.PrimitiveType;
var Buffer = Cesium$e.Buffer;
var BufferUsage = Cesium$e.BufferUsage;
var DrawCommand = Cesium$e.DrawCommand;
var Pass = Cesium$e.Pass;
var RenderState = Cesium$e.RenderState;
var ShaderProgram = Cesium$e.ShaderProgram;
var ShaderSource = Cesium$e.ShaderSource;
var VertexArray = Cesium$e.VertexArray;
var BlendingState = Cesium$e.BlendingState;
var CullFace = Cesium$e.CullFace;
var Material = Cesium$e.Material;
var SceneMode$3 = Cesium$e.SceneMode;
var VertexFormat = Cesium$e.VertexFormat;
var CesiumMath = Cesium$e.Math;
var Matrix3 = Cesium$e.Matrix3;
var Matrix4$1 = Cesium$e.Matrix4;
var JulianDate = Cesium$e.JulianDate; // var BoxGeometry = Cesium.BoxGeometry;
// var EllipsoidGeometry = Cesium.EllipsoidGeometry;

var sin = Math.sin;
var cos = Math.cos;
var tan = Math.tan;
var atan = Math.atan;
var attributeLocations = {
  position: 0,
  normal: 1
};

function ScanRadarPrimitive(options) {
  var self = this;
  options = defaultValue(options, defaultValue.EMPTY_OBJECT);
  /**
   * 是否显示
   */

  this.show = defaultValue(options.show, true);
  /**
   * 切分程度
   */

  this.slice = defaultValue(options.slice, 32);
  /**
   * 传感器的模型矩阵
   */

  this.modelMatrix = Matrix4$1.clone(options.modelMatrix, new Matrix4$1());
  this._modelMatrix = new Matrix4$1();
  this._computedModelMatrix = new Matrix4$1();
  this._computedScanPlaneModelMatrix = new Matrix4$1();
  /**
   * 传感器的半径
   */

  this.radius = defaultValue(options.radius, Number.POSITIVE_INFINITY);
  this._radius = undefined;
  /**
   * 传感器水平半角
   */

  this.xHalfAngle = defaultValue(options.xHalfAngle, 0);
  this._xHalfAngle = undefined;
  /**
   * 传感器垂直半角
   */

  this.yHalfAngle = defaultValue(options.yHalfAngle, 0);
  this._yHalfAngle = undefined;
  /**
   * 线的颜色
   */

  this.lineColor = defaultValue(options.lineColor, Color.WHITE);
  /**
   * 是否显示扇面的线
   */

  this.showSectorLines = defaultValue(options.showSectorLines, true);
  /**
   * 是否显示扇面和圆顶面连接的线
   */

  this.showSectorSegmentLines = defaultValue(options.showSectorSegmentLines, true);
  /**
   * 是否显示侧面
   */

  this.showLateralSurfaces = defaultValue(options.showLateralSurfaces, true);
  /**
   * 目前用的统一材质
   * @type {Material}
   */

  this.material = defined$7(options.material) ? options.material : Material.fromType(Material.ColorType);
  this._material = undefined;
  this._translucent = undefined;
  /**
   * 侧面材质
   * @type {Material}
   */

  this.lateralSurfaceMaterial = defined$7(options.lateralSurfaceMaterial) ? options.lateralSurfaceMaterial : Material.fromType(Material.ColorType);
  this._lateralSurfaceMaterial = undefined;
  this._lateralSurfaceTranslucent = undefined;
  /**
   * 是否显示圆顶表面
   */

  this.showDomeSurfaces = defaultValue(options.showDomeSurfaces, true);
  /**
   * 圆顶表面材质
   * @type {Material}
   */

  this.domeSurfaceMaterial = defined$7(options.domeSurfaceMaterial) ? options.domeSurfaceMaterial : Material.fromType(Material.ColorType);
  this._domeSurfaceMaterial = undefined;
  /**
   * 是否显示圆顶面线
   */

  this.showDomeLines = defaultValue(options.showDomeLines, true);
  /**
   * 是否显示与地球相交的线
   */

  this.showIntersection = defaultValue(options.showIntersection, true);
  /**
   * 与地球相交的线的颜色
   */

  this.intersectionColor = defaultValue(options.intersectionColor, Color.WHITE);
  /**
   * 与地球相交的线的宽度（像素）
   */

  this.intersectionWidth = defaultValue(options.intersectionWidth, 5.0);
  /**
   * 是否穿过地球
   */

  this.showThroughEllipsoid = defaultValue(options.showThroughEllipsoid, false);
  this._showThroughEllipsoid = undefined;
  /**
   * 是否显示扫描面
   */

  this.showScanPlane = defaultValue(options.showScanPlane, true);
  /**
   * 扫描面颜色
   */

  this.scanPlaneColor = defaultValue(options.scanPlaneColor, Color.WHITE);
  /**
   * 扫描面模式 垂直vertical/水平horizontal
   */

  this.scanPlaneMode = defaultValue(options.scanPlaneMode, 'horizontal');
  /**
   * 扫描速率
   */

  this.scanPlaneRate = defaultValue(options.scanPlaneRate, 10);
  this._scanePlaneXHalfAngle = 0;
  this._scanePlaneYHalfAngle = 0; //时间计算的起点

  this._time = JulianDate.now();
  this._boundingSphere = new BoundingSphere$1();
  this._boundingSphereWC = new BoundingSphere$1(); //扇面 sector

  this._sectorFrontCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.TRIANGLES,
    boundingVolume: this._boundingSphereWC
  });
  this._sectorBackCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.TRIANGLES,
    boundingVolume: this._boundingSphereWC
  });
  this._sectorVA = undefined; //扇面边线 sectorLine

  this._sectorLineCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.LINES,
    boundingVolume: this._boundingSphereWC
  });
  this._sectorLineVA = undefined; //扇面分割线 sectorSegmentLine

  this._sectorSegmentLineCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.LINES,
    boundingVolume: this._boundingSphereWC
  });
  this._sectorSegmentLineVA = undefined; //弧面 dome

  this._domeFrontCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.TRIANGLES,
    boundingVolume: this._boundingSphereWC
  });
  this._domeBackCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.TRIANGLES,
    boundingVolume: this._boundingSphereWC
  });
  this._domeVA = undefined; //弧面线 domeLine

  this._domeLineCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.LINES,
    boundingVolume: this._boundingSphereWC
  });
  this._domeLineVA = undefined; //扫描面 scanPlane/scanRadial

  this._scanPlaneFrontCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.TRIANGLES,
    boundingVolume: this._boundingSphereWC
  });
  this._scanPlaneBackCommand = new DrawCommand({
    owner: this,
    primitiveType: PrimitiveType.TRIANGLES,
    boundingVolume: this._boundingSphereWC
  });
  this._scanRadialCommand = undefined;
  this._colorCommands = [];
  this._frontFaceRS = undefined;
  this._backFaceRS = undefined;
  this._sp = undefined;
  this._uniforms = {
    u_type: function u_type() {
      return 0; //面
    },
    u_xHalfAngle: function u_xHalfAngle() {
      return self.xHalfAngle;
    },
    u_yHalfAngle: function u_yHalfAngle() {
      return self.yHalfAngle;
    },
    u_radius: function u_radius() {
      return self.radius;
    },
    u_showThroughEllipsoid: function u_showThroughEllipsoid() {
      return self.showThroughEllipsoid;
    },
    u_showIntersection: function u_showIntersection() {
      return self.showIntersection;
    },
    u_intersectionColor: function u_intersectionColor() {
      return self.intersectionColor;
    },
    u_intersectionWidth: function u_intersectionWidth() {
      return self.intersectionWidth;
    },
    u_normalDirection: function u_normalDirection() {
      return 1.0;
    },
    u_lineColor: function u_lineColor() {
      return self.lineColor;
    }
  };
  this._scanUniforms = {
    u_xHalfAngle: function u_xHalfAngle() {
      return self._scanePlaneXHalfAngle;
    },
    u_yHalfAngle: function u_yHalfAngle() {
      return self._scanePlaneYHalfAngle;
    },
    u_radius: function u_radius() {
      return self.radius;
    },
    u_color: function u_color() {
      return self.scanPlaneColor;
    },
    u_showThroughEllipsoid: function u_showThroughEllipsoid() {
      return self.showThroughEllipsoid;
    },
    u_showIntersection: function u_showIntersection() {
      return self.showIntersection;
    },
    u_intersectionColor: function u_intersectionColor() {
      return self.intersectionColor;
    },
    u_intersectionWidth: function u_intersectionWidth() {
      return self.intersectionWidth;
    },
    u_normalDirection: function u_normalDirection() {
      return 1.0;
    },
    u_lineColor: function u_lineColor() {
      return self.lineColor;
    }
  };
}

ScanRadarPrimitive.prototype.update = function (frameState) {
  var mode = frameState.mode;

  if (!this.show || mode !== SceneMode$3.SCENE3D) {
    return;
  }

  var createVS = false;
  var createRS = false;
  var createSP = false;
  var xHalfAngle = this.xHalfAngle;
  var yHalfAngle = this.yHalfAngle;

  if (xHalfAngle < 0.0 || yHalfAngle < 0.0) {
    throw new DeveloperError$3('halfAngle must be greater than or equal to zero.');
  }

  if (xHalfAngle == 0.0 || yHalfAngle == 0.0) {
    return;
  }

  if (this._xHalfAngle !== xHalfAngle || this._yHalfAngle !== yHalfAngle) {
    this._xHalfAngle = xHalfAngle;
    this._yHalfAngle = yHalfAngle;
    createVS = true;
  }

  var radius = this.radius;

  if (radius < 0.0) {
    throw new DeveloperError$3('this.radius must be greater than or equal to zero.');
  }

  var radiusChanged = false;

  if (this._radius !== radius) {
    radiusChanged = true;
    this._radius = radius;
    this._boundingSphere = new BoundingSphere$1(Cartesian3$3.ZERO, this.radius);
  }

  var modelMatrixChanged = !Matrix4$1.equals(this.modelMatrix, this._modelMatrix);

  if (modelMatrixChanged || radiusChanged) {
    Matrix4$1.clone(this.modelMatrix, this._modelMatrix);
    Matrix4$1.multiplyByUniformScale(this.modelMatrix, this.radius, this._computedModelMatrix);
    BoundingSphere$1.transform(this._boundingSphere, this.modelMatrix, this._boundingSphereWC);
  }

  var showThroughEllipsoid = this.showThroughEllipsoid;

  if (this._showThroughEllipsoid !== this.showThroughEllipsoid) {
    this._showThroughEllipsoid = showThroughEllipsoid;
    createRS = true;
  }

  var material = this.material;

  if (this._material !== material) {
    this._material = material;
    createRS = true;
    createSP = true;
  }

  var translucent = material.isTranslucent();

  if (this._translucent !== translucent) {
    this._translucent = translucent;
    createRS = true;
  }

  if (this.showScanPlane) {
    var time = frameState.time;
    var timeDiff = JulianDate.secondsDifference(time, this._time);

    if (timeDiff < 0) {
      this._time = JulianDate.clone(time, this._time);
    }

    var percentage = Math.max(timeDiff % this.scanPlaneRate / this.scanPlaneRate, 0);
    console.log(percentage);
    var angle;

    if (this.scanPlaneMode == 'horizontal') {
      angle = 2 * yHalfAngle * percentage - yHalfAngle;
      var cosYHalfAngle = cos(angle);
      var tanXHalfAngle = tan(xHalfAngle);
      var maxX = atan(cosYHalfAngle * tanXHalfAngle);
      this._scanePlaneXHalfAngle = maxX;
      this._scanePlaneYHalfAngle = angle;
      Cesium$e.Matrix3.fromRotationX(this._scanePlaneYHalfAngle, matrix3Scratch);
    } else {
      angle = 2 * xHalfAngle * percentage - xHalfAngle;
      var tanYHalfAngle = tan(yHalfAngle);
      var cosXHalfAngle = cos(angle);
      var maxY = atan(cosXHalfAngle * tanYHalfAngle);
      this._scanePlaneXHalfAngle = angle;
      this._scanePlaneYHalfAngle = maxY;
      Cesium$e.Matrix3.fromRotationY(this._scanePlaneXHalfAngle, matrix3Scratch);
    }

    Cesium$e.Matrix4.multiplyByMatrix3(this.modelMatrix, matrix3Scratch, this._computedScanPlaneModelMatrix);
    Matrix4$1.multiplyByUniformScale(this._computedScanPlaneModelMatrix, this.radius, this._computedScanPlaneModelMatrix);
  }

  if (createVS) {
    createVertexArray(this, frameState);
  }

  if (createRS) {
    createRenderState(this, showThroughEllipsoid, translucent);
  }

  if (createSP) {
    createShaderProgram(this, frameState, material);
  }

  if (createRS || createSP) {
    createCommands(this, translucent);
  }

  var commandList = frameState.commandList;
  var passes = frameState.passes;
  var colorCommands = this._colorCommands;

  if (passes.render) {
    for (var i = 0, len = colorCommands.length; i < len; i++) {
      var colorCommand = colorCommands[i];
      commandList.push(colorCommand);
    }
  }
};

var matrix3Scratch = new Matrix3();
var nScratch = new Cartesian3$3(); //region -- VertexArray --

/**
 * 计算zoy面和zoy面单位扇形位置
 * @param primitive
 * @returns {{zoy: Array, zox: Array}}
 */

function computeUnitPosiiton(primitive, xHalfAngle, yHalfAngle) {
  var slice = primitive.slice; //以中心为角度

  var cosYHalfAngle = cos(yHalfAngle);
  var tanYHalfAngle = tan(yHalfAngle);
  var cosXHalfAngle = cos(xHalfAngle);
  var tanXHalfAngle = tan(xHalfAngle);
  var maxY = atan(cosXHalfAngle * tanYHalfAngle);
  var maxX = atan(cosYHalfAngle * tanXHalfAngle); //ZOY面单位圆

  var zoy = [];

  for (var i = 0; i < slice; i++) {
    var phi = 2 * maxY * i / (slice - 1) - maxY;
    zoy.push(new Cartesian3$3(0, sin(phi), cos(phi)));
  } //zox面单位圆


  var zox = [];

  for (var i = 0; i < slice; i++) {
    var phi = 2 * maxX * i / (slice - 1) - maxX;
    zox.push(new Cartesian3$3(sin(phi), 0, cos(phi)));
  }

  return {
    zoy: zoy,
    zox: zox
  };
}
/**
 * 计算扇面的位置
 * @param unitPosition
 * @returns {Array}
 */


function computeSectorPositions(primitive, unitPosition) {
  var xHalfAngle = primitive.xHalfAngle,
      yHalfAngle = primitive.yHalfAngle,
      zoy = unitPosition.zoy,
      zox = unitPosition.zox;
  var positions = []; //zoy面沿y轴逆时针转xHalfAngle

  var matrix3 = Matrix3.fromRotationY(xHalfAngle, matrix3Scratch);
  positions.push(zoy.map(function (p) {
    return Matrix3.multiplyByVector(matrix3, p, new Cesium$e.Cartesian3());
  })); //zox面沿x轴顺时针转yHalfAngle

  var matrix3 = Matrix3.fromRotationX(-yHalfAngle, matrix3Scratch);
  positions.push(zox.map(function (p) {
    return Matrix3.multiplyByVector(matrix3, p, new Cesium$e.Cartesian3());
  }).reverse()); //zoy面沿y轴顺时针转xHalfAngle

  var matrix3 = Matrix3.fromRotationY(-xHalfAngle, matrix3Scratch);
  positions.push(zoy.map(function (p) {
    return Matrix3.multiplyByVector(matrix3, p, new Cesium$e.Cartesian3());
  }).reverse()); //zox面沿x轴逆时针转yHalfAngle

  var matrix3 = Matrix3.fromRotationX(yHalfAngle, matrix3Scratch);
  positions.push(zox.map(function (p) {
    return Matrix3.multiplyByVector(matrix3, p, new Cesium$e.Cartesian3());
  }));
  return positions;
}
/**
 * 创建扇面顶点
 * @param context
 * @param positions
 * @returns {*}
 */


function createSectorVertexArray(context, positions) {
  var planeLength = Array.prototype.concat.apply([], positions).length - positions.length;
  var vertices = new Float32Array(2 * 3 * 3 * planeLength);
  var k = 0;

  for (var i = 0, len = positions.length; i < len; i++) {
    var planePositions = positions[i];
    var n = Cartesian3$3.normalize(Cartesian3$3.cross(planePositions[0], planePositions[planePositions.length - 1], nScratch), nScratch);

    for (var j = 0, planeLength = planePositions.length - 1; j < planeLength; j++) {
      vertices[k++] = 0.0;
      vertices[k++] = 0.0;
      vertices[k++] = 0.0;
      vertices[k++] = -n.x;
      vertices[k++] = -n.y;
      vertices[k++] = -n.z;
      vertices[k++] = planePositions[j].x;
      vertices[k++] = planePositions[j].y;
      vertices[k++] = planePositions[j].z;
      vertices[k++] = -n.x;
      vertices[k++] = -n.y;
      vertices[k++] = -n.z;
      vertices[k++] = planePositions[j + 1].x;
      vertices[k++] = planePositions[j + 1].y;
      vertices[k++] = planePositions[j + 1].z;
      vertices[k++] = -n.x;
      vertices[k++] = -n.y;
      vertices[k++] = -n.z;
    }
  }

  var vertexBuffer = Buffer.createVertexBuffer({
    context: context,
    typedArray: vertices,
    usage: BufferUsage.STATIC_DRAW
  });
  var stride = 2 * 3 * Float32Array.BYTES_PER_ELEMENT;
  var attributes = [{
    index: attributeLocations.position,
    vertexBuffer: vertexBuffer,
    componentsPerAttribute: 3,
    componentDatatype: ComponentDatatype.FLOAT,
    offsetInBytes: 0,
    strideInBytes: stride
  }, {
    index: attributeLocations.normal,
    vertexBuffer: vertexBuffer,
    componentsPerAttribute: 3,
    componentDatatype: ComponentDatatype.FLOAT,
    offsetInBytes: 3 * Float32Array.BYTES_PER_ELEMENT,
    strideInBytes: stride
  }];
  return new VertexArray({
    context: context,
    attributes: attributes
  });
}
/**
 * 创建扇面边线顶点
 * @param context
 * @param positions
 * @returns {*}
 */


function createSectorLineVertexArray(context, positions) {
  var planeLength = positions.length;
  var vertices = new Float32Array(3 * 3 * planeLength);
  var k = 0;

  for (var i = 0, len = positions.length; i < len; i++) {
    var planePositions = positions[i];
    vertices[k++] = 0.0;
    vertices[k++] = 0.0;
    vertices[k++] = 0.0;
    vertices[k++] = planePositions[0].x;
    vertices[k++] = planePositions[0].y;
    vertices[k++] = planePositions[0].z;
  }

  var vertexBuffer = Buffer.createVertexBuffer({
    context: context,
    typedArray: vertices,
    usage: BufferUsage.STATIC_DRAW
  });
  var stride = 3 * Float32Array.BYTES_PER_ELEMENT;
  var attributes = [{
    index: attributeLocations.position,
    vertexBuffer: vertexBuffer,
    componentsPerAttribute: 3,
    componentDatatype: ComponentDatatype.FLOAT,
    offsetInBytes: 0,
    strideInBytes: stride
  }];
  return new VertexArray({
    context: context,
    attributes: attributes
  });
}
/**
 * 创建扇面圆顶面连接线顶点
 * @param context
 * @param positions
 * @returns {*}
 */


function createSectorSegmentLineVertexArray(context, positions) {
  var planeLength = Array.prototype.concat.apply([], positions).length - positions.length;
  var vertices = new Float32Array(3 * 3 * planeLength);
  var k = 0;

  for (var i = 0, len = positions.length; i < len; i++) {
    var planePositions = positions[i];

    for (var j = 0, planeLength = planePositions.length - 1; j < planeLength; j++) {
      vertices[k++] = planePositions[j].x;
      vertices[k++] = planePositions[j].y;
      vertices[k++] = planePositions[j].z;
      vertices[k++] = planePositions[j + 1].x;
      vertices[k++] = planePositions[j + 1].y;
      vertices[k++] = planePositions[j + 1].z;
    }
  }

  var vertexBuffer = Buffer.createVertexBuffer({
    context: context,
    typedArray: vertices,
    usage: BufferUsage.STATIC_DRAW
  });
  var stride = 3 * Float32Array.BYTES_PER_ELEMENT;
  var attributes = [{
    index: attributeLocations.position,
    vertexBuffer: vertexBuffer,
    componentsPerAttribute: 3,
    componentDatatype: ComponentDatatype.FLOAT,
    offsetInBytes: 0,
    strideInBytes: stride
  }];
  return new VertexArray({
    context: context,
    attributes: attributes
  });
}
/**
 * 创建圆顶面顶点
 * @param context
 */


function createDomeVertexArray(context) {
  var geometry = Cesium$e.EllipsoidGeometry.createGeometry(new Cesium$e.EllipsoidGeometry({
    vertexFormat: VertexFormat.POSITION_ONLY,
    stackPartitions: 32,
    slicePartitions: 32
  }));
  var vertexArray = VertexArray.fromGeometry({
    context: context,
    geometry: geometry,
    attributeLocations: attributeLocations,
    bufferUsage: BufferUsage.STATIC_DRAW,
    interleave: false
  });
  return vertexArray;
}
/**
 * 创建圆顶面连线顶点
 * @param context
 */


function createDomeLineVertexArray(context) {
  var geometry = Cesium$e.EllipsoidOutlineGeometry.createGeometry(new Cesium$e.EllipsoidOutlineGeometry({
    vertexFormat: VertexFormat.POSITION_ONLY,
    stackPartitions: 32,
    slicePartitions: 32
  }));
  var vertexArray = VertexArray.fromGeometry({
    context: context,
    geometry: geometry,
    attributeLocations: attributeLocations,
    bufferUsage: BufferUsage.STATIC_DRAW,
    interleave: false
  });
  return vertexArray;
}
/**
 * 创建扫描面顶点
 * @param context
 * @param positions
 * @returns {*}
 */


function createScanPlaneVertexArray(context, positions) {
  var planeLength = positions.length - 1;
  var vertices = new Float32Array(3 * 3 * planeLength);
  var k = 0;

  for (var i = 0; i < planeLength; i++) {
    vertices[k++] = 0.0;
    vertices[k++] = 0.0;
    vertices[k++] = 0.0;
    vertices[k++] = positions[i].x;
    vertices[k++] = positions[i].y;
    vertices[k++] = positions[i].z;
    vertices[k++] = positions[i + 1].x;
    vertices[k++] = positions[i + 1].y;
    vertices[k++] = positions[i + 1].z;
  }

  var vertexBuffer = Buffer.createVertexBuffer({
    context: context,
    typedArray: vertices,
    usage: BufferUsage.STATIC_DRAW
  });
  var stride = 3 * Float32Array.BYTES_PER_ELEMENT;
  var attributes = [{
    index: attributeLocations.position,
    vertexBuffer: vertexBuffer,
    componentsPerAttribute: 3,
    componentDatatype: ComponentDatatype.FLOAT,
    offsetInBytes: 0,
    strideInBytes: stride
  }];
  return new VertexArray({
    context: context,
    attributes: attributes
  });
}

function createVertexArray(primitive, frameState) {
  var context = frameState.context;
  var unitSectorPositions = computeUnitPosiiton(primitive, primitive.xHalfAngle, primitive.yHalfAngle);
  var positions = computeSectorPositions(primitive, unitSectorPositions); //显示扇面

  if (primitive.showLateralSurfaces) {
    primitive._sectorVA = createSectorVertexArray(context, positions);
  } //显示扇面线


  if (primitive.showSectorLines) {
    primitive._sectorLineVA = createSectorLineVertexArray(context, positions);
  } //显示扇面圆顶面的交线


  if (primitive.showSectorSegmentLines) {
    primitive._sectorSegmentLineVA = createSectorSegmentLineVertexArray(context, positions);
  } //显示弧面


  if (primitive.showDomeSurfaces) {
    primitive._domeVA = createDomeVertexArray(context);
  } //显示弧面线


  if (primitive.showDomeLines) {
    primitive._domeLineVA = createDomeLineVertexArray(context);
  } //显示扫描面


  if (primitive.showScanPlane) {
    if (primitive.scanPlaneMode == 'horizontal') {
      var unitScanPlanePositions = computeUnitPosiiton(primitive, CesiumMath.PI_OVER_TWO, 0);
      primitive._scanPlaneVA = createScanPlaneVertexArray(context, unitScanPlanePositions.zox);
    } else {
      var unitScanPlanePositions = computeUnitPosiiton(primitive, 0, CesiumMath.PI_OVER_TWO);
      primitive._scanPlaneVA = createScanPlaneVertexArray(context, unitScanPlanePositions.zoy);
    }
  }
} //endregion
//region -- ShaderProgram --


function createCommonShaderProgram(primitive, frameState, material) {
  var context = frameState.context;
  var vs = shader2;
  var fs = new ShaderSource({
    sources: [shader3, material.shaderSource, shader1]
  });
  primitive._sp = ShaderProgram.replaceCache({
    context: context,
    shaderProgram: primitive._sp,
    vertexShaderSource: vs,
    fragmentShaderSource: fs,
    attributeLocations: attributeLocations
  });
  var pickFS = new ShaderSource({
    sources: [shader3, material.shaderSource, shader1],
    pickColorQualifier: 'uniform'
  });
  primitive._pickSP = ShaderProgram.replaceCache({
    context: context,
    shaderProgram: primitive._pickSP,
    vertexShaderSource: vs,
    fragmentShaderSource: pickFS,
    attributeLocations: attributeLocations
  });
}

function createScanPlaneShaderProgram(primitive, frameState, material) {
  var context = frameState.context;
  var vs = shader2;
  var fs = new ShaderSource({
    sources: [shader3, material.shaderSource, shader4]
  });
  primitive._scanePlaneSP = ShaderProgram.replaceCache({
    context: context,
    shaderProgram: primitive._scanePlaneSP,
    vertexShaderSource: vs,
    fragmentShaderSource: fs,
    attributeLocations: attributeLocations
  });
}

function createShaderProgram(primitive, frameState, material) {
  createCommonShaderProgram(primitive, frameState, material);

  if (primitive.showScanPlane) {
    createScanPlaneShaderProgram(primitive, frameState, material);
  }
} //endregion
//region -- RenderState --


function createRenderState(primitive, showThroughEllipsoid, translucent) {
  if (translucent) {
    primitive._frontFaceRS = RenderState.fromCache({
      depthTest: {
        enabled: !showThroughEllipsoid
      },
      depthMask: false,
      blending: BlendingState.ALPHA_BLEND,
      cull: {
        enabled: true,
        face: CullFace.BACK
      }
    });
    primitive._backFaceRS = RenderState.fromCache({
      depthTest: {
        enabled: !showThroughEllipsoid
      },
      depthMask: false,
      blending: BlendingState.ALPHA_BLEND,
      cull: {
        enabled: true,
        face: CullFace.FRONT
      }
    });
    primitive._pickRS = RenderState.fromCache({
      depthTest: {
        enabled: !showThroughEllipsoid
      },
      depthMask: false,
      blending: BlendingState.ALPHA_BLEND
    });
  } else {
    primitive._frontFaceRS = RenderState.fromCache({
      depthTest: {
        enabled: !showThroughEllipsoid
      },
      depthMask: true
    });
    primitive._pickRS = RenderState.fromCache({
      depthTest: {
        enabled: true
      },
      depthMask: true
    });
  }
} //endregion
//region -- Command --


function createCommand(primitive, frontCommand, backCommand, frontFaceRS, backFaceRS, sp, va, uniforms, modelMatrix, translucent, pass, isLine) {
  if (translucent && backCommand) {
    backCommand.vertexArray = va;
    backCommand.renderState = backFaceRS;
    backCommand.shaderProgram = sp;
    backCommand.uniformMap = combine(uniforms, primitive._material._uniforms);

    backCommand.uniformMap.u_normalDirection = function () {
      return -1.0;
    };

    backCommand.pass = pass;
    backCommand.modelMatrix = modelMatrix;

    primitive._colorCommands.push(backCommand);
  }

  frontCommand.vertexArray = va;
  frontCommand.renderState = frontFaceRS;
  frontCommand.shaderProgram = sp;
  frontCommand.uniformMap = combine(uniforms, primitive._material._uniforms);

  if (isLine) {
    frontCommand.uniformMap.u_type = function () {
      return 1;
    };
  }

  frontCommand.pass = pass;
  frontCommand.modelMatrix = modelMatrix;

  primitive._colorCommands.push(frontCommand);
}

function createCommands(primitive, translucent) {
  primitive._colorCommands.length = 0;
  var pass = translucent ? Pass.TRANSLUCENT : Pass.OPAQUE; //显示扇面

  if (primitive.showLateralSurfaces) {
    createCommand(primitive, primitive._sectorFrontCommand, primitive._sectorBackCommand, primitive._frontFaceRS, primitive._backFaceRS, primitive._sp, primitive._sectorVA, primitive._uniforms, primitive._computedModelMatrix, translucent, pass);
  } //显示扇面线


  if (primitive.showSectorLines) {
    createCommand(primitive, primitive._sectorLineCommand, undefined, primitive._frontFaceRS, primitive._backFaceRS, primitive._sp, primitive._sectorLineVA, primitive._uniforms, primitive._computedModelMatrix, translucent, pass, true);
  } //显示扇面交接线


  if (primitive.showSectorSegmentLines) {
    createCommand(primitive, primitive._sectorSegmentLineCommand, undefined, primitive._frontFaceRS, primitive._backFaceRS, primitive._sp, primitive._sectorSegmentLineVA, primitive._uniforms, primitive._computedModelMatrix, translucent, pass, true);
  } //显示弧面


  if (primitive.showDomeSurfaces) {
    createCommand(primitive, primitive._domeFrontCommand, primitive._domeBackCommand, primitive._frontFaceRS, primitive._backFaceRS, primitive._sp, primitive._domeVA, primitive._uniforms, primitive._computedModelMatrix, translucent, pass);
  } //显示弧面线


  if (primitive.showDomeLines) {
    createCommand(primitive, primitive._domeLineCommand, undefined, primitive._frontFaceRS, primitive._backFaceRS, primitive._sp, primitive._domeLineVA, primitive._uniforms, primitive._computedModelMatrix, translucent, pass, true);
  } //显示扫描面


  if (primitive.showScanPlane) {
    createCommand(primitive, primitive._scanPlaneFrontCommand, primitive._scanPlaneBackCommand, primitive._frontFaceRS, primitive._backFaceRS, primitive._scanePlaneSP, primitive._scanPlaneVA, primitive._scanUniforms, primitive._computedScanPlaneModelMatrix, translucent, pass);
  }
}

var originalDefaultVisualizersCallback = Cesium$e.DataSourceDisplay.defaultVisualizersCallback;

Cesium$e.DataSourceDisplay.defaultVisualizersCallback = function (scene, entityCluster, dataSource) {
  var entities = dataSource.entities;
  var array = originalDefaultVisualizersCallback(scene, entityCluster, dataSource);
  return array.concat([new ScanRadarVisualizer(scene, entities)]);
};



var sensor = /*#__PURE__*/Object.freeze({
  __proto__: null,
  CamberRadarPrimitive: CamberRadarPrimitive,
  CylinderRadarPrimitive: CylinderRadarPrimitive,
  ScanRadarGraphics: ScanRadarGraphics
});

var PopupTooltipTool = /*#__PURE__*/function () {
  /**
   * @param {Cesium.Viewer} viewer 
   */
  function PopupTooltipTool(viewer) {
    _classCallCheck(this, PopupTooltipTool);

    this.viewer = viewer;
    /**
     * @property {Boolean} [toolOpen=true] 是否开启弹窗工具
     */

    this.toolOpen = true;
    this.popupHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this.tooltipHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this.defaultVal = {
      type: 2,
      show: true
    };
    this.lastTooltipObj = undefined;
  }
  /**
   * 根据像素坐标拾取对象
   * @param {Cesium.Cartesian2} px 
   * @returns {Object} 
   */


  _createClass(PopupTooltipTool, [{
    key: "getPickobj",
    value: function getPickobj(px) {
      var pick = this.viewer.scene.pick(px);
      if (!pick) return undefined;
      var tileFeature, obj;

      if (pick.id && pick.id instanceof Cesium.Entity) {
        // entity
        obj = pick.id;
      } else if (pick.primitive && !(pick.primitive instanceof Cesium.Cesium3DTileset)) {
        // 普通primitive
        obj = pick.primitive;
      } else if (pick.primitive && pick.primitive instanceof Cesium.Cesium3DTileset) {
        // 拾取3dtiles模型
        if (pick instanceof Cesium.Cesium3DTileFeature) {
          // 3dtiles，精细化模型
          tileFeature = pick;
        }

        obj = pick.primitive;
      }

      return {
        obj: obj,
        tileFeature: tileFeature
      }; // 如果拾取的是3dtiles单个瓦片 则返回当前点击的瓦片 否则为空
    }
    /**
     * 绑定点击弹出气泡窗
     */

  }, {
    key: "bindPopup",
    value: function bindPopup() {
      var _this = this;

      this.popupHandler.setInputAction(function (evt) {
        //单击开始绘制
        if (!_this.toolOpen) return;

        var res = _this.getPickobj(evt.position);

        if (!res || !res.obj) return;
        var obj = res.obj;
        if (!obj || !obj.popup) return;

        if (!obj.popupPrompt) {
          // 未创建气泡窗 则创建
          // 当前对象未创建气泡窗
          var popup = {};

          if (typeof obj.popup == "string") {
            popup.content = obj.popup;
          } else {
            popup = Object.assign(popup, obj.popup);
          }

          popup.type = popup.type || 2; // 点击弹窗默认为固定点位弹窗

          obj.popupPrompt = _this.createPrompt(obj, popup, evt.position);
        } else {
          if (obj.isPointType) {
            // 点状 直接打开
            obj.popupPrompt.setVisible(true);
          } else {
            // 线状 先销毁原气泡窗 再在新位置重新构建气泡窗
            obj.popupPrompt.destroy();
            obj.popupPrompt = undefined;
            var _popup = {};

            if (typeof obj.popup == "string") {
              _popup.content = obj.popup;
            } else {
              _popup = Object.assign(_popup, obj.popup);
            }

            _popup.type = _popup.type || 2; // 点击弹窗默认为固定点位弹窗

            obj.popupPrompt = _this.createPrompt(obj, _popup, evt.position);
          }
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
    }
    /**
     * 绑定鼠标移入气泡窗
     */

  }, {
    key: "bindTooltip",
    value: function bindTooltip() {
      var _this2 = this;

      this.popupHandler.setInputAction(function (evt) {
        //单击开始绘制
        var res = _this2.getPickobj(evt.endPosition);

        if (!res || !res.obj) {
          // 移出对象后 删除
          if (_this2.lastTooltipObj && _this2.lastTooltipObj.tooltipPrompt) {
            _this2.lastTooltipObj.tooltipPrompt.destroy();

            _this2.lastTooltipObj.tooltipPrompt = undefined;
            _this2.lastTooltipObj = undefined;
          }

          return;
        } else {
          // 移入对象 
          var obj = res.obj; // 前后两次移入的对象不一致时 清除上一次移入的对象气泡窗

          if (_this2.lastTooltipObj && obj.tooltipId != _this2.lastTooltipObj.tooltipId) {
            if (_this2.lastTooltipObj.tooltipPrompt) {
              _this2.lastTooltipObj.tooltipPrompt.destroy();

              _this2.lastTooltipObj.tooltipPrompt = undefined;
            }

            _this2.lastTooltipObj = undefined;
          }

          if (!obj.tooltip) return;

          if (!obj.tooltipPrompt) {
            var popup = {};

            if (typeof obj.tooltip == "string") {
              popup.content = obj.tooltip;
            } else {
              popup = Object.assign(popup, obj.tooltip);
            }

            popup.type = popup.type || 2; // 点击弹窗默认为固定点位弹窗

            obj.tooltipPrompt = _this2.createPrompt(obj, popup, evt.endPosition);
          } else {
            obj.tooltipPrompt.update({
              x: evt.endPosition.x,
              y: evt.endPosition.y - 4
            });
          }

          obj.tooltipId = new Date().getTime() + "" + Math.floor(Math.random() * 10000) + "" + Math.floor(Math.random() * 10000);
          _this2.lastTooltipObj = obj;
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
    /**
     * 创建气泡窗
     * @param {Cesium.Primitive} obj 
     * @param {Object} promptAttr 
     * @param {Cesium.Cartesian2} px 
     * @returns 
     */

  }, {
    key: "createPrompt",
    value: function createPrompt(obj, promptAttr, px) {
      var position;
      var defaultVal = JSON.parse(JSON.stringify(this.defaultVal));

      if (obj instanceof Cesium.Entity) {
        // 实体
        if (obj.billboard || obj.point || obj.model) {
          // 点状对象
          var ent = obj.billboard || obj.point || obj.model;
          position = obj.position.getValue(this.viewer.clock.currentTime);
          var isClamp = ent.heightReference.getValue();

          if (isClamp == 1) {
            var ctgc = Cesium.Cartographic.fromCartesian(position);
            var height = this.viewer.scene.sampleHeight(ctgc.clone());

            if (height == undefined) {
              // 自动计算值失败 通过拾取 重新计算高度，加个偏移参数，防止pickPosition拾取时undefined
              var newPosition = this.viewer.scene.pickPosition({
                x: px.x + 1,
                y: px.y + 1
              });
              var newCtgc = Cesium.Cartographic.fromCartesian(newPosition);
              height = newCtgc.height;
            } else {
              ctgc.height = height;
            }

            position = Cesium.Cartographic.toCartesian(ctgc);
          }

          obj.isPointType = true; // 点状对象
        }

        if (obj.polyline || obj.polygon || obj.rectangle || obj.ellipse || obj.plane || obj.path) {
          // 非点状对象
          // 1、重新根据px来拾取位置
          position = this.viewer.scene.pickPosition(px);
          obj.isPointType = false; // 点状对象
        }
      }

      if (obj.tooltip) {
        this.defaultVal.closeBtn = false;
      }

      defaultVal.position = position;

      if (promptAttr.constructor == String) {
        // 支持两种传参 字符串 / 对象
        defaultVal.content = promptAttr;
      } else {
        defaultVal = Object.assign(defaultVal, promptAttr);
      }

      return new Prompt$1(this.viewer, defaultVal);
    }
    /**
     * 关闭弹窗
     */

  }, {
    key: "close",
    value: function close() {
      this.toolOpen = false;
    }
    /**
     * 打开弹窗
     */

  }, {
    key: "open",
    value: function open() {
      this.toolOpen = true;
    }
  }]);

  return PopupTooltipTool;
}();

var CustomHandler = /*#__PURE__*/function () {
  function CustomHandler(viewer) {
    _classCallCheck(this, CustomHandler);

    this.viewer = viewer;
    this.clickHandler = undefined;
    this.mouseMoveHandler = undefined;
    this.rightClickHandler = undefined;
    this.lastobj_click = undefined;
    this.lastobj_mouseMove = undefined;
    this.lastobj_rightClick = undefined;
  }

  _createClass(CustomHandler, [{
    key: "click",
    value: function click() {
      var _this = this;

      if (!this.clickHandler) this.clickHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.clickHandler.setInputAction(function (evt) {
        var res = _this.getPickobj(evt.position);

        if (!res) {
          // 未拾取到对象
          _this.pickNull(evt, _this.lastobj_click, 'unClick');

          _this.lastobj_click = undefined;
          return;
        } else {
          // 拾取到对象
          _this.pickDiff(evt, res, _this.lastobj_click, 'click', 'unClick');

          _this.lastobj_click = res;
        }
      }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
    }
  }, {
    key: "mouseMove",
    value: function mouseMove() {
      var _this2 = this;

      if (!this.mouseMoveHandler) this.mouseMoveHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.mouseMoveHandler.setInputAction(function (evt) {
        var res = _this2.getPickobj(evt.endPosition);

        if (!res) {
          // 未拾取到对象
          _this2.pickNull(evt, _this2.lastobj_mouseMove, 'unMouseMove');

          _this2.lastobj_mouseMove = undefined;
          return;
        } else {
          // 拾取到对象
          _this2.pickDiff(evt, res, _this2.lastobj_mouseMove, 'mouseMove', 'unMouseMove');

          _this2.lastobj_mouseMove = res;
        }
      }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);
    }
  }, {
    key: "rightClick",
    value: function rightClick() {
      var _this3 = this;

      if (!this.rightClickHandler) this.rightClickHandler = new Cesium.ScreenSpaceEventHandler(this.viewer.scene.canvas);
      this.rightClickHandler.setInputAction(function (evt) {
        var res = _this3.getPickobj(evt.position);

        if (!res) {
          // 未拾取到对象
          _this3.pickNull(evt, _this3.lastobj_rightClick, 'unRightClick');

          _this3.lastobj_rightClick = undefined;
          return;
        } else {
          // 拾取到对象
          _this3.pickDiff(evt, res, _this3.lastobj_rightClick, 'rightClick', 'unRightClick');

          _this3.lastobj_rightClick = res;
        }
      }, Cesium.ScreenSpaceEventType.RIGHT_CLICK);
    }
    /**
     * 销毁
     */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.clickHandler) {
        this.clickHandler.destroy();
        this.clickHandler = undefined;
      }

      if (this.mouseMoveHandler) {
        this.mouseMoveHandler.destroy();
        this.mouseMoveHandler = undefined;
      }

      if (this.rightClickHandler) {
        this.rightClickHandler.destroy();
        this.rightClickHandler = undefined;
      }

      this.lastobj_click = undefined;
      this.lastobj_mouseMove = undefined;
      this.lastobj_rightClick = undefined;
    }
    /**
     * 根据像素坐标拾取对象
     * @param {Cesium.Cartesian2} px 
     * @returns {Object} 
     */

  }, {
    key: "getPickobj",
    value: function getPickobj(px) {
      var pick = this.viewer.scene.pick(px);
      if (!pick) return undefined; // 目前点击对象时  分为多个类型 

      var tileFeature, obj;

      if (pick.id && pick.id instanceof Cesium.Entity) {
        // entity
        obj = pick.id;
      } else if (pick.primitive && !(pick.primitive instanceof Cesium.Cesium3DTileset)) {
        // 普通primitive
        obj = pick.primitive;
      } else if (pick.primitive && pick.primitive instanceof Cesium.Cesium3DTileset) {
        // 拾取3dtiles模型
        if (pick instanceof Cesium.Cesium3DTileFeature) {
          // 3dtiles，精细化模型
          tileFeature = pick;
        }

        obj = pick.primitive;
      }

      obj.randomId = this.getRandomId();
      return {
        obj: obj,
        tileFeature: tileFeature
      }; // 如果拾取的是3dtiles单个瓦片 则返回当前点击的瓦片 否则为空
    }
    /**
     * 当前拾取为空
     */

  }, {
    key: "pickNull",
    value: function pickNull(evt, lastpick, pickType) {
      // 未拾取到对象
      if (lastpick && lastpick.obj && lastpick.obj[pickType]) lastpick.obj[pickType](evt, lastpick.obj);
    }
    /**
     * 拾取前后对象不同
     */

  }, {
    key: "pickDiff",
    value: function pickDiff(evt, res, lastpick, pickType, unpickType) {
      if (lastpick) {
        // 存在上一个对象 先执行上一个对象的相关操作
        if (lastpick.obj.randomId != res.obj.randomId) {
          // 1、拾取到的对象不同
          if (lastpick && lastpick.obj[unpickType]) lastpick.obj[unpickType](evt, lastpick.obj);
        } else {
          // 2、拾取到的对象相同 但拾取的是同一个对象的不同瓦片
          if (res.tileFeature && lastpick.tileFeature && res.tileFeature._batchId != lastpick.tileFeature._batchId) {
            if (lastpick && lastpick.obj[unpickType]) lastpick.obj[unpickType](evt, lastpick.obj);
          }
        }
      }

      if (res.obj[pickType]) res.obj[pickType](evt, res.obj);
    }
    /**
     * 随机生成id
     */

  }, {
    key: "getRandomId",
    value: function getRandomId() {
      return new Date().getTime() + "" + Math.floor(Math.random() * 10000) + "" + Math.floor(Math.random() * 10000);
    }
  }]);

  return CustomHandler;
}();

/**
 * 定义mapViewer.opt参数
 * @typedef {Object} mapViewer.opt
 * 构建参数
 * @property {Object} map 地图相关基础参数
 * @property {String} baseServer 基础地址，配置后，所有url中的`${baseServer}`将被此参数替代
 * @property {Array} baseLayers 地图底图图层配置
 * @property {Array} operateLayers 地图业务图层配置
 */

/**
 * 地图viewer对象类
 * @class 
 */

var MapViewer = /*#__PURE__*/function () {
  /**
   * 
   * @param {String} domId 地图div容器id
   * @param {mapViewer.opt} opt 
   * @param {Object} opt.map 地图配置
   * @example
   *  const mapConfig = {
          baseServer: "http://localhost:1119/",
          map: {
              cameraView:{ // 初始化视角
              "x" : 117.11652300702349,
              "y" : 31.822531554698113,
              "z" : 249.22424831865297,
              "heading" : 92.73801048659865,
              "pitch" : -78.63126631978243,
              "roll" : 359.9999069885447,
              "duration" : 0
              },
              errorRender: false, // 是否开启崩溃刷新
              debugShowFramesPerSecond : false, // 是否显示帧数
              worldAnimate: false,
              lnglatNavigation: true, // 经纬度及相机位置提示
              rightTool: true, // 是否开启右键功能
              popupTooltipTool: true, // 是否开启气泡窗
              navigationTool: true, // 导航球及比例尺
              depthTestAgainstTerrain: true, // 是否开启深度监测
              viewer: { // 同Cesium.viewer中配置
                  animation: false,
                  baseLayerPicker: false,
                  fullscreenButton: false,
                  geocoder: false,
                  homeButton: false,
                  infoBox: false,
                  sceneModePicker: false,
                  selectionIndicator: false,
                  timeline: false,
                  navigationHelpButton: false,
                  scene3DOnly: true,
                  useDefaultRenderLoop: true,
                  showRenderLoopErrors: false,
                  terrainExaggeration: 1,
              },
              terrain: {
                  url: "http://data.marsgis.cn/terrain",
                  show: true,
              },
          },
          baseLayers: [
              {
                  name: "单张地图",
                  type: "singleImage",
                  url: "./vis3d/images/layer/world.jpg",
                  iconImg: "./vis3d/images/layer/world.jpg",
                  show: false,
                  alpha: 1,
                  rectangle: [-180, -90, 180, 90],
              }
          ],
          operateLayers: [
              {
                  name: "测试图层",
                  type: "group",
                  open: true,
                  children: [
                      {
                          name: "天地图",
                          type: "tdt",
                          layerName: "img",
                          show: false,
                          key: "a217b99b7be68b98104548d78e9a679a",
                          compare: true,
                      },
                      {
                          name: "单张地图",
                          type: "singleImage",
                          url: "./vis3d/images/layer/world.jpg",
                          iconImg: "./vis3d/images/layer/world.jpg",
                          show: false,
                          layerSplit: true,
                          alpha: 1,
                          rectangle: [-180, -90, 180, 90],
                      },
                      {
                          name: "全国地图（深色）",
                          type: "xyz",
                          show: false,
                          url: "http://8.142.20.247:25548/layer/chengdu/{z}/{x}/{y}.png",
                      }
                  ],
              },
              {
                  name: "三维模型",
                  type: "group",
                  open: true,
                  children: [
                      {
                          name: "城区模型",
                          type: "3dtiles",
                          url: "http://8.141.58.76:6814/data/3dtiles/tileset.json",
                          show: false,
                          center: {
                              z: 45,
                          },
                          maximumScreenSpaceError: 1,
                      }
                  ],
              },
          ],
          };    
      let mapViewer = new vis3d.MapViewer(
          "mapContainer",
          mapConfig
      ));
   * 
   */
  function MapViewer(domId, opt, success) {
    _classCallCheck(this, MapViewer);

    if (!domId) return;
    /**
     * @property {String} div容器id
     */

    this.domId = domId;
    this.opt = opt || {};
    /**
    * @property {Cesium.Viewer} 地图viewer对象
    */

    this._viewer = null;
    /**
     * @property {LayerTool} baseLayerTool 底图图层控制器
     */

    this.baseLayerTool = null;
    /**
    * @property {LayerTool} operateLayerTool 业务图层控制器
    */

    this.operateLayerTool = null;
    this.operatePlotTool = null;
    /**
    * @property {RightTool}  rightTool 右键菜单工具
    */

    this.rightTool = null;
    /**
    * @property {LatlngNavigation}  lnglatNavigation 底图坐标提示工具
    */

    this.lnglatNavigation = null;
    /**
     * @property {Navigation} compassTool 指北针
     */

    this.compassTool = null;
    /**
     * @property {PopupTooltipTool}  popupTooltipTool 鼠标提示工具
     */

    this.popupTooltipTool = null;
    this._viewer = this.createViewer();
    this.loadTerrain(this.opt.map.terrain.url, this.opt.map.terrain.show);
    this.loadbaseLayers();
    this.loadOperateLayers();
    if (this.opt.map.lnglatNavigation) this.openLnglatNavigation();
    if (this.opt.map.rightTool) this.openRightTool(); //  if (this.opt.map.popupTooltipTool) this.openPopupTooltip();

    this.openPopupTooltip();
    if (this.opt.map.navigationTool) this.openNavigationTool();

    if (this.opt.map.worldAnimate) {
      this.openWorldAnimate();
    } else {
      if (this.opt.map.cameraView) util$1.setCameraView(this.opt.map.cameraView, this._viewer);
    }

    if (this.opt.map.errorRender) {
      this._viewer.scene.renderError.addEventListener(function () {
        window.location.reload();
      }, this);
    } // 开启窗口大小监听

    /*  if(this.opt.map.openSizeListener){
         this.openSizeListener();
     } */


    this._viewer.scene.debugShowFramesPerSecond = this.opt.map.debugShowFramesPerSecond; // 亮度设置

    if (this.opt.map.brightness != undefined) {
      var stages = this._viewer.scene.postProcessStages;
      this._viewer.scene.brightness = this._viewer.scene.brightness || stages.add(Cesium.PostProcessStageLibrary.createBrightnessStage());
      this._viewer.scene.brightness.enabled = true;
      this._viewer.scene.brightness.uniforms.brightness = Number(this.opt.map.brightness);
    }

    var autoBindHandler = this.opt.autoBindHandler == undefined ? true : this.opt.autoBindHandler;

    if (autoBindHandler) {
      var ch = new CustomHandler(this._viewer);
      ch.click();
      ch.mouseMove();
      ch.rightClick();
    }

    if (success) success(this);
  }

  _createClass(MapViewer, [{
    key: "viewer",
    get: function get() {
      return this._viewer;
    } // 构建地图

  }, {
    key: "createViewer",
    value: function createViewer() {
      var viewerAttr = this.opt.map.viewer;
      var viewer = new window.Cesium.Viewer(this.domId, viewerAttr); // 移除原来影像

      viewer.imageryLayers.removeAll(); // 是否展示cesium官方logo

      viewer._cesiumWidget._creditContainer.style.display = "none";
      viewer.mapConfig = this.opt;
      viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK); // 是否开启深度检测

      viewer.scene.globe.depthTestAgainstTerrain = this.opt.map.depthTestAgainstTerrain;
      return viewer;
    } // 构建图层

  }, {
    key: "loadbaseLayers",
    value: function loadbaseLayers() {
      var baseLayers = this.opt.baseLayers;
      var baseServer = this.opt.baseServer || "";
      if (!baseLayers) return;
      if (!this.baseLayerTool) this.baseLayerTool = new LayerTool(this._viewer);

      for (var i = 0; i < baseLayers.length; i++) {
        var layer = baseLayers[i];

        if (!layer.type) {
          console.log("缺少基础图层的图层类型", layer);
          return;
        }

        if (layer.type == "group") continue; // 添加自定义id

        layer.id = layer.id || new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0);
        if (layer.url) layer.url = layer.url.replace("${baseServer}", baseServer);
        this.baseLayerTool.add(layer);
      }
    } // 构建业务图层

  }, {
    key: "loadOperateLayers",
    value: function loadOperateLayers() {
      var operateLayers = this.opt.operateLayers;
      if (!operateLayers) return; // 递归查到所有的图层

      var allOperateLayers = [];

      function dg(layers) {
        for (var i = 0; i < layers.length; i++) {
          var layer = layers[i]; // 添加id

          layer.id = layer.id || new Date().getTime() + "" + Number(Math.random() * 1000).toFixed(0);
          layer.alpha = layer.alpha == undefined ? 1 : layer.alpha;

          if (layer.children && layer.children.length > 0) {
            dg(layer.children);
          } else {
            allOperateLayers.push(layer);
          }
        }
      }

      dg(operateLayers);
      var baseServer = this.opt.baseServer || "";

      for (var i = 0; i < allOperateLayers.length; i++) {
        var layer = allOperateLayers[i];

        if (!layer.type) {
          console.log("缺少基础图层的图层类型", layer);
          return;
        }

        if (layer.type == "group") continue;

        if (layer.type == "plot" && layer.show) {
          // 兼容单个类型标绘在文件中配置
          if (!this.operatePlotTool) {
            this.operatePlotTool = new DrawTool(this._viewer, {
              canEdit: false
            });
          }

          layer.type = layer.plotType;
          this.operatePlotTool.createByPositions(layer);
        } else {
          if (layer.url) layer.url = layer.url.replace("${baseServer}", baseServer);
          if (!this.operateLayerTool) this.operateLayerTool = new LayerTool(this._viewer);
          this.operateLayerTool.add(layer);
        }
      }
    }
    /**
     * 加载地形
     * @param {String} url 地形路径地址
    */

  }, {
    key: "loadTerrain",
    value: function () {
      var _loadTerrain = _asyncToGenerator( /*#__PURE__*/_regeneratorRuntime().mark(function _callee(url) {
        var terrainProvider;
        return _regeneratorRuntime().wrap(function _callee$(_context) {
          while (1) {
            switch (_context.prev = _context.next) {
              case 0:
                if (url) {
                  _context.next = 2;
                  break;
                }

                return _context.abrupt("return");

              case 2:
                // 移除原地形
                this._viewer.scene.terrainProvider = new Cesium.EllipsoidTerrainProvider({});
                _context.next = 5;
                return Cesium.CesiumTerrainProvider.fromUrl(url);

              case 5:
                terrainProvider = _context.sent;
                this._viewer.scene.terrainProvider = terrainProvider;

              case 7:
              case "end":
                return _context.stop();
            }
          }
        }, _callee, this);
      }));

      function loadTerrain(_x) {
        return _loadTerrain.apply(this, arguments);
      }

      return loadTerrain;
    }()
    /**
     * 设置地形的显示隐藏
     * @param {Boolean} visible true显示 / false隐藏
    */

  }, {
    key: "setTerrainVisible",
    value: function () {
      var _setTerrainVisible = _asyncToGenerator( /*#__PURE__*/_regeneratorRuntime().mark(function _callee2(visible) {
        var _this$opt$map$terrain;

        return _regeneratorRuntime().wrap(function _callee2$(_context2) {
          while (1) {
            switch (_context2.prev = _context2.next) {
              case 0:
                if (visible) {
                  _context2.next = 6;
                  break;
                }

                _context2.next = 3;
                return new Cesium.EllipsoidTerrainProvider({});

              case 3:
                this._viewer.scene.terrainProvider = _context2.sent;
                _context2.next = 7;
                break;

              case 6:
                this.loadTerrain((_this$opt$map$terrain = this.opt.map.terrain) === null || _this$opt$map$terrain === void 0 ? void 0 : _this$opt$map$terrain.url);

              case 7:
                this._viewer.scene.render();

              case 8:
              case "end":
                return _context2.stop();
            }
          }
        }, _callee2, this);
      }));

      function setTerrainVisible(_x2) {
        return _setTerrainVisible.apply(this, arguments);
      }

      return setTerrainVisible;
    }()
    /**
     * 开启右键工具
     */

  }, {
    key: "openRightTool",
    value: function openRightTool() {
      if (!this.rightTool) {
        this.rightTool = new RightTool(this._viewer, {});
      }
    }
    /**
     * 关闭右键工具
     */

  }, {
    key: "closeRightTool",
    value: function closeRightTool() {
      if (this.rightTool) {
        this.rightTool.destroy();
        this.rightTool = null;
      }
    }
    /**
     * 开启鼠标提示
     */

  }, {
    key: "openPopupTooltip",
    value: function openPopupTooltip() {
      if (!this.popupTooltip) {
        this.popupTooltip = new PopupTooltipTool(this._viewer, {});
        this.popupTooltip.bindTooltip();
        this.popupTooltip.bindPopup();
      }
    }
    /**
    * 关闭鼠标提示
    */

  }, {
    key: "closePopupTooltip",
    value: function closePopupTooltip() {
      if (this.popupTooltip) {
        this.popupTooltip.destroy();
        this.popupTooltip = undefined;
      }
    }
    /**
     * 开启底部坐标提示
     */

  }, {
    key: "openLnglatNavigation",
    value: function openLnglatNavigation() {
      if (!this.lnglatNavigation) this.lnglatNavigation = new LnglatTool(this._viewer);
    }
    /**
    * 关闭底部坐标提示
    */

  }, {
    key: "closeLnglatNavigation",
    value: function closeLnglatNavigation() {
      if (this.lnglatNavigation) {
        this.lnglatNavigation.destroy();
        this.lnglatNavigation = null;
      }
    }
  }, {
    key: "openWorldAnimate",
    value: function openWorldAnimate() {
      var that = this;
      this.setRotate({
        x: this.opt.map.cameraView.x,
        y: this.opt.map.cameraView.y
      }, function () {
        if (that.opt.map.cameraView) {
          that.opt.map.cameraView.duration = 3;
          util$1.setCameraView(that.opt.map.cameraView, that.viewer);
        }
      });
    }
  }, {
    key: "setRotate",
    value: function setRotate(obj, callback) {
      //传入所需定位的经纬度 及旋转的速度 旋转的圈数
      if (!obj.x || !obj.y) {
        console.log("设定地球旋转时，并未传入经纬度！");
        return;
      }

      var v = obj.v || 1;
      var i = 0;
      var q = obj.q || 2;
      var x = obj.x;
      var y = obj.y;
      var z = obj.z;
      var that = this;
      var interVal = window.setInterval(function () {
        x = x + v;

        if (x >= 179) {
          x = -180;
          i++;
        }

        that.viewer.scene.camera.setView({
          destination: new Cesium.Cartesian3.fromDegrees(x, y, z || 20000000)
        });

        if (i == q) {
          //此处有瑕疵  未修改
          clearInterval(interVal);
          callback();
        }
      }, 10);
    }
  }, {
    key: "openNavigationTool",
    value: function openNavigationTool() {
      this.compassTool = new CesiumNavigation(this._viewer, {
        enableCompass: true,
        // 罗盘

        /* compass: {
            style: {
                top: "120px",
                left: "120px"
            }
        }, */
        enableZoomControls: true,
        // 缩放控制器
        enableDistanceLegend: true,
        // 比例尺

        /*  distanceLegend: {
             style: {
                 top: "120px",
                 left: "120px"
             }
         }, */
        enableCompassOuterRing: true,
        // 罗盘外环
        view: this._viewer.mapConfig.map && this._viewer.mapConfig.map.cameraView
      });
    }
    /**
     * 销毁
    */

  }, {
    key: "destroy",
    value: function destroy() {
      if (this.baseLayerTool) {
        this.baseLayerTool.destroy();
        this.baseLayerTool = null;
      }

      if (this.operateLayerTool) {
        this.operateLayerTool.destroy();
        this.operateLayerTool = null;
      }

      if (this.operatePlotTool) {
        this.operatePlotTool.destroy();
        this.operatePlotTool = null;
      }

      if (this.lnglatNavigation) {
        this.lnglatNavigation.destroy();
        this.lnglatNavigation = null;
      }

      if (this._viewer) {
        this._viewer.destroy();

        this._viewer = null;
      }
    }
    /**
     * 添加地图窗口大小监听
     * @param {Function} callback 
     */

  }, {
    key: "openSizeListener",
    value: function openSizeListener(callback) {
      var that = this;
      var obdom = document.getElementById(this.domId);
      var MutationObserver = window.MutationObserver || window.webkitMutationObserver || window.MozMutationObserver;
      var mutationObserver = new MutationObserver(function (mutations) {
        var width = window.getComputedStyle(obdom).getPropertyValue('width');
        var height = window.getComputedStyle(obdom).getPropertyValue('height');
        width = window.parseInt(width);
        height = window.parseInt(height);

        if (that.lnglatNavigation) {
          var res = width > 1000;
          that.lnglatNavigation.setVisible(res);
        }

        if (that.compassTool) {
          that.compassTool.setVisible(height > 300);
        }

        if (callback) callback(width, height);
      });
      mutationObserver.observe(obdom, {
        childList: false,
        // 子节点的变动（新增、删除或者更改）
        attributes: true,
        // 属性的变动
        characterData: false,
        // 节点内容或节点文本的变动
        subtree: false // 是否将观察器应用于该节点的所有后代节点

      });
    }
  }]);

  return MapViewer;
}();

var plot = {
  Tool: DrawTool,
  Billboard: CreateBillboard,
  Circle: CreateCircle,
  Model: CreateGltfModel,
  Label: CreateLabel,
  Point: CreatePoint,
  Polygon: CreatePolygon,
  Rectangle: CreateRectangle,
  Polyline: CreatePolyline,
  Arrow: CreateArrow
};
var layer = {
  Tool: LayerTool,
  Arcgiscache: ArcgiscacheLayer,
  Mapserver: MapserverLayer,
  Grid: GridLayer,
  Geojson: GeojsonLayer,
  TDT: TDTLayer,
  Image: SingleImageLayer,
  TMS: TMSLayer,
  XYZ: XYZLayer,
  Tileset: TilesetLayer,
  WMS: WMSLayer,
  WMTS: WMTSLayer,
  urltemplate: UrltemplateLayer,
  Baidu: BaiduLayer,
  Tencent: TencentLayer,
  OSM: OSMLayer,
  layerload: layerload
};
var measure = {
  Tool: MeasureTool,
  Azimutht: MeasureAzimutht,
  GroundDistance: MeasureGroundDistance,
  Height: MeasureHeight,
  Lnglat: MeasureLnglat,
  Triangle: MeasureTriangle,
  SpaceDistance: MeasureSpaceDistance,
  SpaceArea: MeasureSpaceArea
};
var roam = {
  Tool: RoamTool,
  Roam: Roam
};
var primitive = {
  Circle: CustomCriclePrimitive,
  Wall: CustomWallPrimitive,
  Cylinder: CustomCylinderPrimitive,
  Cone: ConePrimitive
};
var graphic = {
  TrackCylinder: TrackCylinderGraphic
};
var weather = {
  fog: fog,
  snow: snow,
  rain: rain
};
var common = {
  Navigation: CesiumNavigation,
  ZoomTool: ZoomTool,
  RightTool: RightTool,
  LnglatTool: LnglatTool,
  LayerSplit: LayerSplit,
  Cluster: Cluster,
  Prompt: Prompt$1,
  SkyboxGround: SkyboxGround,
  // OverviewMap: OverviewMap,
  selectModel: selectModel
};
var view = {
  viewAround: viewAround,
  viewPoint: viewPoint,
  worldRotate: worldRotate
};
var tileset = {
  Clip: TilesetClip,
  Edit: TilesetEdit,
  PointLight: PointLight,
  Cut: TilesetCut,
  Flat: Flat
};
var analysis = {
  Sunshine: Sunshine,
  LimitHeight: LimitHeight,
  VisualFieldTool: visualFieldTool
};
var query = {
  GaodePOI: GaodePOI,
  GaodeRoute: GaodeRoute
};
var cover = {
  // echarts: echartMap,
  // mapv: mapv,
  Heatmap: Heatmap,
  Heatmap3d: Heatmap3d
};
var VERSION = '1.1.1';
var vis3d_export = {
  VERSION: VERSION,
  MapViewer: MapViewer,
  sensor: sensor,
  layer: layer,
  material: material,
  layerload: layerload,
  util: util$1,
  tool: tool,
  plot: plot,
  draw: draw,
  measure: measure,
  roam: roam,
  primitive: primitive,
  graphic: graphic,
  weather: weather,
  common: common,
  view: view,
  tileset: tileset,
  analysis: analysis,
  query: query,
  // cover: cover
};

export default vis3d_export;
