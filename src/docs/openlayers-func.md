# 常用方法
## 移动结束获取缩放层级
~~~js
this.map.on('moveend', (e)=> {
    let zoom = this.map.getView().getZoom()
     console.log(zoom)
})
~~~
## 点击获取坐标
~~~js
this.map.on('click', (e) => {
    console.log(e.coordinate);  
})
~~~
## 移动轨迹
~~~js
humanTracking() {
      this.map.removeLayer(this.map.getLayers().array_.find(item => item.className_ == 'human'))
      this.map.removeLayer(this.map.getLayers().array_.find(item => item.className_ == 'roadLine'))
      this.map.getView().animate({ zoom: 16, center: [121.53356596604773, 25.03654660340881] })
      const coordinate = [
        [121.5316274290011, 25.038811249491314],
        [121.53159119466379, 25.03832208593749],
        [121.53280504496401, 25.03823150009419],
        [121.53276881062669, 25.03696329828799],
        [121.53427253562548, 25.03696329828799],
        [121.53429552689978, 25.03655733224487],
        [121.53356596604773, 25.03654660340881],
        [121.5335840832164, 25.035423338951883],
        [121.53470734767332, 25.03527840160260],
        [121.5345805274927, 25.033774676603820],
        [121.53762146607825, 25.033660546508784],
        [121.5376858390946, 25.033317223754878],
        [121.54077659917444, 25.03323116154402]
      ];
      const geometry = new LineString(coordinate)
      //添加移动线

      const featureLine = new Feature({
        geometry: geometry,
      });
      const lineStyle = new Style({
        stroke: new Stroke({
          color: "#53AA08",
          width: 8
        }),
        fill: new Fill({
          color: '#fff'
        }),
      })
      featureLine.setStyle(lineStyle)
      let lineSource = new VectorSource()
      lineSource.addFeature(featureLine)
      let lineLayer = new VectorLayer({
        className: "roadLine"
      })
      lineLayer.setSource(lineSource)
      this.map.addLayer(lineLayer)

      const traceSource = new VectorSource()
      const vectorLayer = new VectorLayer({
        className: 'human',
        source: traceSource,
      });
      let lastTime, distance
      const createMarkAnimation = (lineFeature) => {
        let personGeo = new Point(lineFeature.getGeometry().getFirstCoordinate());
        const markFeature = new Feature({
          type: 'moveMark',
          geometry: personGeo
        });
        const markStyle = new Style({
          image: new Icon({
            anchor: [0.5, 0.9],
            scale: 0.2,
            src: human	//---自行设置移动元素点的图片路径
          })
        });
        markFeature.setStyle(markStyle);
        traceSource.addFeature(markFeature);
        this.map.addLayer(vectorLayer);
        // 开始路径动画
        lastTime = Date.now();
        distance = 0;
        vectorLayer.on('postrender', e => {
          startRunAnimation(this.map, e, 100, lineFeature, personGeo, markStyle);
        });
      }
      const startRunAnimation = (map_ol, e, speed, feature, geometry, style) => {
        const time = e.frameState.time;
        const elapsedTime = time - lastTime;
        distance = (distance + (speed * elapsedTime) / 1e6) % 2;
        if (distance >= 1) distance = 0; //从头重新开始运动
        lastTime = time;
        const currentCoordinate = feature.getGeometry().getCoordinateAt(distance > 1 ? 2 - distance : distance);
        geometry.setCoordinates(currentCoordinate);
        const vectorContext = getVectorContext(e);
        vectorContext.setStyle(style);
        vectorContext.drawGeometry(geometry);
        //告诉OpenLayers继续postrender动画
        map_ol.render();
      }
      createMarkAnimation(featureLine)
    },
~~~