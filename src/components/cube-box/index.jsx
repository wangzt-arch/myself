import React from "react";
import "./index.scss";

function CubeBox() {
    return (
        <div className="cube-box">
            <div className="cube-box__right">React</div>
            <div className="cube-box__left">Taro</div>
            <div className="cube-box__top">工程化</div>
            <div className="cube-box__bottom">Vue</div>
            <div className="cube-box__center">技术栈</div>
        </div>
    );
}

export default CubeBox;
