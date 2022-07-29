import React from "react";
import { useEffect } from "react";
import { seriesData } from "./seriesData";
import * as echarts from "echarts";
import china from "./china.json";
import Header from "../../components/Header";
import "./index.scss";

function YqDistribution() {
  useEffect(() => {
    echarts.registerMap("china", JSON.stringify(china));
    const myCharts = echarts.init(document.getElementById("my-charts"));
    const option = {
      geo: {
        // 作为底图，设置地图外围边框
        id: "china",
        map: "china",
        roam: true,
        show: true,
        selectedMode: true,
        label: {
          color: "transparent",
        },
        itemStyle: {
          borderColor: "#000",
          borderWidth: 0.5,
        },
        emphasis: {
          // 高亮状态下的样式-鼠标悬停
          label: {
            color: "transparent",
          },
          itemStyle: {
            areaColor: "rgb(82, 148, 241)",
            opacity: 1,
          },
        },
        select: {
          // 选中时样式
          label: {
            color: "transparent",
          },
          itemStyle: {
            areaColor: "rgb(82, 148, 241)",
          },
        },
        tooltip: {
          show: false,
        },
      },
      series: [
        {
          type: "scatter",
          name: "城市散点标记",
          coordinateSystem: "geo",
          label: {
            show: true,
            position: "top",
            color: "transparent",
            fontSize: 12,
            fontWeight: "bolder",
            formatter: function (params) {
              const name = params.data.name;
              return [`{name|${name}}`].join("  ");
            },
            rich: {
              name: {
                fontSize: 12,
                color: "blue",
              },
            },
          },
          data: seriesData,
        },
      ],
      tooltip: {
        trigger: "item",
        padding: 0,
        backgroundColor: "transparent",
        borderColor: "transparent",
      },
    };
    myCharts.setOption(option);
  }, []);
  return (
    <div className="yq-distribution">
      <Header></Header>
      <div className="my-charts" id="my-charts"></div>
    </div>
  );
}

export default YqDistribution;
