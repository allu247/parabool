import React, { Component } from 'react'
import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import RotationDirections from './components/RotationDirections';

class Buttons extends Component {
  render() {
    const { history } = this.props;

    return (
      <div>
        <AppBar position="static">
          <Tabs value={0}>
            <Tab label="Button Controlls" onClick={() => history.push('/buttons')} />
            <Tab label="Controller" onClick={() => history.push('/controller')} />
            <Tab label="Information" onClick={() => history.push('/register')} />
          </Tabs>
        </AppBar>
        <div className="controller">
            <RotationDirections />
        </div>
      </div>
    );
  }
}

export default Buttons;