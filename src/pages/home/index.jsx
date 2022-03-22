import React, { Component } from "react";
import "./index.css";

export default class Home extends Component {
  goto = (e) => {
    this.props.history.push(`/${e}`);
  };
  render() {
    return (
      <div className="home" onClick={() => this.goto("account")}>
        home
      </div>
    );
  }
}
