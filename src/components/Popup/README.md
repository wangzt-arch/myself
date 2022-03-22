# Popup

搜索框弹出层组件 

## 示例代码
弹出层

```tsx
<Popup
      isOpened={isOpened}
      onClose={() => this.onInternalPopupClose()}
      overlayStyle="right-filter"
>
    <Filter/>
</Popup>
```

弹出层带二级弹出子项

```tsx
   <Popup
    isOpened={isOpened}
     onClose={() => this.onInternalPopupClose()}
      >
   {this.renderPopup()}
	</Popup>
```

```tsx
  renderPopup = () => {
    const { popup_current } = this.state;
    return ['综合排序', '按新品排序', '按人气排序'].map((item, i) => {
      return (
        <PopupItem
          className={classNames({
            'popup-item': true,
            'popup-item__active': popup_current === i
          })}
          key={i}
          index={i}
          onClick={this.onInternalPopupItemClick}
        >
          {item}
        </PopupItem>
      );
    });
  };
```



## PopupProps

| 参数          | 类型                                 | 必填 | 说明                                  |
| ------------- | ------------------------------------ | ---- | :------------------------------------ |
| isOpened     | boolean                              | 必填 | 是否展示弹出层内元素                  |
| onClose       | onClose?: (*name*?: string) => void; | 否   | 元素关闭出发的事件                    |
| style         | CSSProperties                        | 否   | 弹出层样式。 更改弹出层定位方式及距离 |
| overlayStyle | 'top'  'right-filter'  'action-sheet'| 否   | 弹出层弹出方向                        |
| actionSheetTitle | string           | 否   | 底部弹出得标题文字                        |
| onOverlayClick | onOverlayClick?: () => void           | 否   | 遮罩层触发函数                        |
| backIsOpen | boolean         | 否   | 是否显示回退按钮                        |
| onBackButton | onBackButton?: () => void         | 否   | 回退按钮的点击事件                        |

