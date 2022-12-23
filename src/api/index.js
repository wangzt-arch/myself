import axios from "axios";

export function getTranslate(text) {
  return axios({
    method: 'post',
    url: 'http://api.interpreter.caiyunai.com/v1/translator',
    headers: {
      'X-Authorization': 'token 8ediw9sn9qe3ckupgcpq',
      'content-type': 'application/json',
    },
    data: {
      source: text,
      trans_type: "auto2zh",
      detect: "true",
      request_id: "demo"
    }
  })
}