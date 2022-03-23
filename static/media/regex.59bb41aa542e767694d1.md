## 1、手机号码的校验

```js
const phoneReg = /^[1][3-9][0-9]{9}$/
```

## 2、身份证的校验

```js
const sfzReg = /^[1-9]\d{5}(18|19|([23]\d))\d{2}((0[1-9])|(10|11|12))(([0-2][1-9])|10|20|30|31)\d{3}[0-9Xx]$/
```

## 3、邮箱的校验

```js
const emailReg = /^([A-Za-z0-9_\-\.])+\@([A-Za-z0-9_\-\.])+\.([A-Za-z]{2,4})$/
```

## 4、URL的校验

```js
const urlReg = /^((https?|ftp|file):\/\/)?([\da-z\.-]+)\.([a-z\.]{2,6})([\/\w \.-]*)*\/?$/

const urlStr1 = 'https://haha.sunshine.com/xxx/xxx'
console.log(urlReg.test(urlStr1)) // true

const urlStr2 = 'sss://haha.sunshine.com/xxx/xxx'
console.log(urlReg.test(urlStr2)) // false
复制代码
```

## 5、IPv4的校验

```js
const ipv4Reg = /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/
```

## 6、16进制颜色的校验

```js
const color16Reg = /^#?([a-fA-F0-9]{6}|[a-fA-F0-9]{3})$/
```

## 7、日期 YYYY-MM-DD

```js
const dateReg = /^\d{4}(\-)\d{1,2}\1\d{1,2}$/
```

## 8、日期 YYYY-MM-DD hh:mm:ss

```js
const dateReg = /^(\d{1,4})(-|\/)(\d{1,2})\2(\d{1,2}) (\d{1,2}):(\d{1,2}):(\d{1,2})$/
```

## 9、整数的校验

```js
const intReg = /^[-+]?\d*$/
```

## 10、小数的校验

```js
const floatReg = /^[-\+]?\d+(\.\d+)?$/
```

## 11、保留n位小数

```js
function checkFloat(n) {
  return new RegExp(`^([1-9]+[\d]*(.[0-9]{1,${n}})?)$`)
}
```

## 12、邮政编号的校验

```js
const postalNoReg = /^\d{6}$/
```

## 13、QQ号的校验

说明：5-11位数字

```js
const qqReg = /^[1-9][0-9]{4,10}$/
```

## 14、微信号的校验

说明：6至20位，以字母开头，字母，数字，减号，下划线

```js
const wxReg = /^[a-zA-Z]([-_a-zA-Z0-9]{5,19})+$/
```

## 15、车牌号的校验

```js
const carNoReg = /^[京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼使领A-Z]{1}[A-Z]{1}[A-Z0-9]{4}[A-Z0-9挂学警港澳]{1}$/
```

## 16、只含字母的字符串

```js
const letterReg = /^[a-zA-Z]+$/
```

## 17、包含中文的字符串

```js
const cnReg = /[\u4E00-\u9FA5]/
```

## 18、密码强度的校验

说明：密码中必须包含字母、数字、特称字符，至少8个字符，最多30个字符

```js
const passwordReg = /(?=.*[0-9])(?=.*[a-zA-Z])(?=.*[^a-zA-Z0-9]).{8,30}/
```

## 19、字符串长度n的校验

```js
function checkStrLength(n) {
  return new RegExp(`^.{${n}}$`)
}
```

## 20、文件拓展名的校验

```js
function checkFileName (arr) {
  arr = arr.map(name => `.${name}`).join('|')
  return new RegExp(`(${arr})$`)
}
```

## 21、匹配img和src

```js
const imgReg = /<img.*?src=[\"|\']?(.*?)[\"|\']?\s.*?>/ig
```

## 22、匹配html中的注释

```js
const noteReg = /<!--(.*?)-->/g
```

## 23、匹配html中的style

```js
const styleReg = /style="[^=>]*"([(\s+\w+=)|>])/g
```

## 24、匹配html中的颜色

```js
const colorReg = /#([A-Fa-f0-9]{6}|[A-Fa-f0-9]{3})/g
```

## 25、匹配htmlTag（html标签）

```js
const endReg = /<("[^"]*"|'[^']*'|[^'">])*>/g
```