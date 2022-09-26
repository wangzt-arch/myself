# Skills
## taro
多端实现方案，一套代码。兼容多端
* 微信小程序
* 支付宝小程序
* h5
## react
* class
* function

## vue2、3
* 选项式Api
* 组合式Api
* 路由
* ElementUi
* VueX
  * actions  可异步 store.dispatch()
  * mutations 同步提交 store.commit()
* Vue3+jsx
~~~jsx
  import { defineComponent, onMounted, ref } from "vue";
  export default defineComponent({
    setup() {
      onMounted(() => {
      console.log("jsx");
    })
      const renderHeader = (isShow = true) => {
        return <div v-show={isShow}>header</div>
    }
      const msg = ref('message+jsx+vue')
        return { msg, renderHeader }
  },

  render() {
    return <div>
      {this.renderHeader(false)}
      {this.renderHeader()}
      <div v-show={false}>{this.msg}</div>
      <div v-show={true}>{this.msg}</div>
    </div>
  }
})
~~~

## webpack

## vite

## axios
* 请求拦截器
* 响应拦截器

## openLayers
* 行进轨迹追踪
* 图层信息采集、标绘
* 聚合

## cesium
* 航拍
* 标绘封装
## ECharts
* 柱状图
* 折线图
* 饼图
* 散点图
* 地图
