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