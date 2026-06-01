import React from "react";
import "./index.scss";

function WztButton(props) {
    const { text = "点击", active = false, onNavigate } = props;

    return (
        <button
            className={active ? "wzt-button wzt-button--active" : "wzt-button"}
            type="button"
            aria-current={active ? "page" : undefined}
            onClick={onNavigate}
        >
            {text}
        </button>
    );
}

export default WztButton;
