import React from "react";
import "./index.scss";

function WztButton(props) {
    const { text = '点击', onNavigate } = props
    return (
        <div className="wzt-button" onClick={() => onNavigate()}>
            {text}
        </div>
    );
}

export default WztButton;
