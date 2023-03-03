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