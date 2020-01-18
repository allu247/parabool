import React, { Component } from 'react'
import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableHead from '@material-ui/core/TableHead';
import TableRow from '@material-ui/core/TableRow';
import Switch from '@material-ui/core/Switch';
import TextField from '@material-ui/core/TextField';
import Button from '@material-ui/core/Button';

export default class Commands extends Component {
  state = {
    checkedA: false,
    checkedB: false,
    frequency: '',
  };

  handleChange(node, type, name) {
    const newValue = !this.state[node];
    const newState = this.state;
    newState[node] = newValue;

    this.setState(newState)
    const value = newValue ? '1' : '0';

    fetch('http://localhost:5000/' + type + '/' + name, {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ value })
    });
  }

  handleFrequencyChange(event, value) {
    event.preventDefault();
    fetch('http://localhost:5000/register/0002', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ value })
    });
  }

  render() {
    const { checkedA, checkedB, frequency } = this.state;

    return (
      <div>
        <h1>Basic commands</h1>
        <Table>
          <TableHead>
            <TableRow>
              <TableCell>Option</TableCell>
              <TableCell>Example</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            <TableRow>
              <TableCell>Turn on motor</TableCell>
              <TableCell>1: Run, 0: Stop (valid when A002 = 03)</TableCell>
              <TableCell>
                <Switch
                  checked={checkedA}
                  onChange={() => this.handleChange('checkedA', 'coil', '0001')}
                  value="checkedA"
                />
              </TableCell>
            </TableRow>
            <TableRow>
              <TableCell>Rotate motor</TableCell>
              <TableCell>1: Reverse rotation, 0: Forward rotation (valid when A002 = 03)</TableCell>
              <TableCell>
                <Switch
                  checked={checkedB}
                  onChange={() => this.handleChange('checkedB', 'coil', '0002')}
                  value="checkedB"
                />
              </TableCell>
            </TableRow>
          </TableBody>
        </Table>
        <h1>Speed</h1>
        <form className="form" onSubmit={(event) => this.handleFrequencyChange(event, frequency)}>
          <TextField
            id="standard-name"
            label="Value"
            value={this.state.frequency}
            onChange={(event) => this.setState({ frequency: event.target.value })}
            margin="normal"
          />
          <Button className="register-submit-button" variant="contained" type="submit">Submit</Button>
        </form>
      </div>
    )
  }
}
