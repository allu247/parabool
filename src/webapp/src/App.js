import React, { Component } from 'react';
import { BrowserRouter as Router, Route } from "react-router-dom";
import { Redirect } from 'react-router-dom'
import Registers from './pages/registers';
import Register from './pages/register';
import Controller from './pages/controller';



class App extends Component {
  render() {
    return (
      <Router>
        <div>
          <Route exact path="/" component={() => <Redirect to="/register" />} />
          <Route path="/register" component={Registers} />
          <Route path="/controller" component={Controller} />
          <Route exact path="/register/:type/:name" component={Register} />
        </div>
      </Router>
    );
  }
}

export default App;
