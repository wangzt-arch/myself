import React from "react";
import { Route, Switch } from "react-router-dom";
import Home from "../pages/home/index";
import Account from "../pages/account/index";

export default () => {
  return (
    <Switch>
      <Route exact path="/" component={Home} />
      <Route path="/home" component={Home} />
      <Route path="/account" component={Account} />
    </Switch>
  );
};
