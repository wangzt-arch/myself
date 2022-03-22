import axios from "axios";
export function getSinglePoetry() {
  return axios.get(
    "https://www.fastmock.site/mock/cb23a99bb5407349c72eb1c00866b330/wzt/api/react"
  );
}
/**
 * 获取天气信息
 * @param {number} cityCode
 * @returns
 */
export default function getWeather(cityCode) {
  const key = "9c49ff55128adf0fc7480c2978dbcbce",
    city = cityCode;
  return axios.get(
    `https://restapi.amap.com/v3/weather/weatherInfo?city=${city}&key=${key}`
  );
}
/**
 * music
 */
export function getMusic(keywords) {
  return axios.get(
    `http://mobilecdn.kugou.com/api/v3/search/song?keyword=${keywords}&page=1&pagesize=10`
  );
}
