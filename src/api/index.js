import axios from "axios";

export function getTranslate(text, language) {
  return axios({
    method: 'post',
    url: 'https://api.interpreter.caiyunai.com/v1/translator',
    headers: {
      'X-Authorization': 'token 8ediw9sn9qe3ckupgcpq',
      'content-type': 'application/json',
    },
    data: {
      source: text,
      trans_type: language === 'en' ? "auto2en" : "auto2zh",
      detect: "true",
      request_id: "demo"
    }
  })
}

const PUSHPLUS_TOKEN = "42d9a046a6374aec90135995ff4cca00";

export function sendMessage(data) {
  return axios({
    method: 'post',
    url: 'https://www.pushplus.plus/send',
    headers: {
      'Content-Type': 'application/json',
    },
    data: {
      token: PUSHPLUS_TOKEN,
      title: `新留言 - ${data.name}`,
      content: `姓名：${data.name}\n联系方式：${data.contact}\n留言：${data.message}`,
      template: 'txt',
      channel: 'wechat'
    }
  })
}