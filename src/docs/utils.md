# 1.移动端禁止手指操作缩放

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

# 2.判断pc或者移动端环境

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

# 3.判断页面上滚还是下滚

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

# 4.判断手指上滑还是下滑

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

