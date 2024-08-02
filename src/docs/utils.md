## 1.移动端禁止手指操作缩放

~~~ js
 //禁止双击放大
 document.documentElement.addEventListener('ondblclick', function (event) {
    if (event.touches.length > 1) {
        event.preventDefault();
    }
}, false);
//禁止手指放大
document.documentElement.addEventListener('touchstart', function (event) {
    if (event.touches.length > 1) {
        event.preventDefault();
    }
}, {
    passive: false
})
~~~

## 2.判断pc或者移动端环境

~~~ js
 function isPC() {
            const userAgentInfo = navigator.userAgent
            console.log(userAgentInfo);
            const Agents = ['Android', 'iPhone', 'SymbianOS', 'Windows Phone', 'iPad', 'iPod']
            let flag = true
            for (var v = 0; v < Agents.length; v++) {
                if (userAgentInfo.indexOf(Agents[v]) > 0) {
                    console.log('mobile');
                }
                else {
                    console.log('pc');
                }
            }
        }
~~~

## 3.判断页面上滚还是下滚

~~~ js
let headerP = 0,
      headerT = 0;
    $(window).scroll(function (e) {
      headerP = $(this).scrollTop();

      if (headerT <= headerP || headerP == 0) {
        console.log("下滚");
      } else {
        console.log("上滚");
      }
      headerT = headerP; //更新上一次scrollTop的值
    });
~~~

## 4.判断手指上滑还是下滑

~~~ js
 $('body').on('touchstart', function (e) {
            scrollStartY = e.originalEvent.changedTouches[0].pageY;
            // console.log(scrollStartY);
        })
        $('body').on('touchmove', function (e) {
            scrollEndY = e.originalEvent.changedTouches[0].pageY;
            // console.log(scrollEndY - this.scrollStartY);
            if (scrollEndY - scrollStartY < 0) {
                console.log('上滑');
            }
            else{
                console.log('下滑');
            }
        })
~~~
## 滚动条
~~~ js
 &::-webkit-scrollbar {
    width: 6px;
    height: 6px;
    background-color: #f1f1f1;
    border-radius: 5px;
  }

  &::-webkit-scrollbar-thumb {
    width: 8px;
    background-color: #a8a8a8;
    border-radius: 5px;
  }
~~~
## 下载pdf文件流
~~~ js
const url = window.URL.createObjectURL(new Blob([res], { type: 'application/pdf' }));
        const link = document.createElement('a');
        link.href = url;
        link.setAttribute('download', val.bookName);
        document.body.appendChild(link);
        link.click()
        document.body.removeChild(link)
~~~
## 下载方法
~~~js
/**
 * 常用工具、和地图无关的方法
 * @exports tool
 * @alias tool
 */
var tool = {};
/**
 * 文本或json等下载方法
 * @param {String} fileName 文件名称，后缀需要加类型，如.txt / .json等
 * @param {String} datastr 文本字符串
 * @example tool.downloadFile("测试.json",JSON.stringify(data));
*/

tool.downloadFile = function (fileName, datastr) {
  var blob = new Blob([datastr]);

  _download(fileName, blob);
};
/**
 * 图片下载方法
 * @param {String} fileName 图片名称
 * @param {Canvas} canvas dom canvas对象
*/


tool.downloadImage = function (fileName, canvas) {
  var base64 = canvas.toDataURL("image/png");
  var blob = base64Img2Blob(base64);

  _download(fileName + '.png', blob);
};

function _download(fileName, blob) {
  var aLink = document.createElement('a');
  aLink.download = fileName;
  aLink.href = URL.createObjectURL(blob);
  document.body.appendChild(aLink);
  aLink.click();
  document.body.removeChild(aLink);
}

function base64Img2Blob(code) {
  var parts = code.split(';base64,');
  var contentType = parts[0].split(':')[1];
  var raw = window.atob(parts[1]);
  var rawLength = raw.length;
  var uInt8Array = new Uint8Array(rawLength);

  for (var i = 0; i < rawLength; ++i) {
    uInt8Array[i] = raw.charCodeAt(i);
  }

  return new Blob([uInt8Array], {
    type: contentType
  });
}
~~~ 
