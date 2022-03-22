import React from "react";
import "./index.scss";

export default function WztInput(props) {
  return (
    <div>
      <input type="text" value={props.title} />
      <input type="number" />
      <input type="password" />
    </div>
  );
}
