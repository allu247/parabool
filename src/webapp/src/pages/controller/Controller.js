import React, { Component } from 'react'
import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import Commands from './components/Commands';
import Settings from './components/Settings';
import Rotate from './components/Rotate';

class Controller extends Component {
  render() {
    const { history } = this.props;

    return (
      <div>
        <AppBar position="static">
          <Tabs value={0}>
            <Tab label="Controller" onClick={() => history.push('/controller')} />
            <Tab label="Information" onClick={() => history.push('/register')} />
          </Tabs>
        </AppBar>
        <div className="controller">
          <Settings />
          <Commands />
          <Rotate />
        </div>
      </div>
    );
  }
}

export default Controller;