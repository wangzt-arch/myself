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