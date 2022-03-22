import React, { Component, CSSProperties } from "react";
import classNames from "classnames";
import "./index.scss";

interface PopupProps {
  /**
   * 是否展示元素
   * @default false
   */
  isOpened: boolean;
  /**
   * 元素被关闭触发的事件
   */
  onClose?: (name?: string) => void;
  overlayStyle?: "top" | "right-filter" | "action-sheet" | "model";
  actionSheetTitle?: string;
  /**
   * 自定义 top 弹框
   */
  style?: CSSProperties;
  /**
   * 遮罩层触发函数
   */
  onOverlayClick?: () => void;
  /**
   * 回退按钮
   */
  backIsOpen?: boolean;
  onBackButton?: () => void;
}
interface PopupState {
  show: boolean;
}
export default class Popup extends Component<PopupProps, PopupState> {
  static defaultProps = {
    overlayStyle: "top",
  };
  constructor(props: PopupProps) {
    super(props);
    const { isOpened } = props;
    this.state = {
      show: isOpened,
    };
  }
  static getDerivedStateFromProps(props: PopupProps, state: PopupState) {
    const { isOpened } = props;
    if (isOpened !== state.show || state.show === false) {
      if (!isOpened) {
        return { show: false };
      } else {
      }
      return { show: isOpened };
    }
    return null;
  }
  /**
   * 阻止手指触摸移动 的冒泡
   */
  onInternalMove = (e: any) => {
    e.stopPropagation();
    e.preventDefault();
  };
  onInternalOverlay = () => {
    const { onClose } = this.props;
    if (onClose !== undefined) {
      onClose();
    }
  };
  onInternalOverlayClick = () => {
    // const { onOverlayClick } = this.props;
    // if (isFunction(onOverlayClick)) {
    //   onOverlayClick();
    // }
  };
  renderActionSheetTitle = () => {
    const {
      overlayStyle,
      actionSheetTitle,
      backIsOpen,
      onBackButton,
    } = this.props;
    return (
      overlayStyle === "action-sheet" && (
        <div
          className={classNames({
            "popup__container-title": true,
          })}
        >
          {backIsOpen && (
            <div>
              <img
                className="popup__container-title__back-icon"
                src="https://b-puzhehei-cdn.co-mall.net/cart/icon%EF%BC%8Fback%402x.png"
                onClick={onBackButton}
                alt=""
              ></img>
            </div>
          )}
          <div>
            <text className="popup__container-title__text">
              {actionSheetTitle}
            </text>
          </div>
          <div>
            <img
              className="popup__container-close"
              src="https://b-puzhehei-cdn.co-mall.net/products-detail/btn_close_popup.png"
              onClick={this.onInternalOverlay}
              alt=""
            />
          </div>
        </div>
      )
    );
  };
  render() {
    const { show } = this.state;
    const { overlayStyle, style } = this.props;
    return (
      <div
        className={classNames({
          popup: !style,
          popup__style: style,
          popup__active: show,
          popup__top: overlayStyle === "top",
          popup__model: overlayStyle === "model",
        })}
        style={style}
        onTouchMove={this.onInternalMove}
        onClick={this.onInternalOverlayClick}
      >
        <div
          className={classNames({
            popup__overlay: true,
          })}
          onClick={this.onInternalOverlay}
        ></div>
        <div
          className={classNames({
            popup__container: true,
            "popup__container--right-filter": overlayStyle === "right-filter",
            "popup__container--action-sheet": overlayStyle === "action-sheet",
          })}
          onTouchMove={this.onInternalMove}
        >
          {this.renderActionSheetTitle()}
          {this.props.children}
        </div>
      </div>
    );
  }
}
