import React from 'react';
import { withRouter } from "react-router-dom";
import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableHead from '@material-ui/core/TableHead';
import TableRow from '@material-ui/core/TableRow';
import Paper from '@material-ui/core/Paper';

const RegisterItems = ({ items, header, history, type }) => (
  <div>
    <h1>{header}</h1>
    <Paper>
      <Table>
        <TableHead>
          <TableRow>
            <TableCell>Register address</TableCell>
            <TableCell align="left">Description</TableCell>
            <TableCell align="left">Example</TableCell>
            <TableCell align="left">Write access</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {items.map(item => (
            <TableRow className="register-item" key={item.name} onClick={() => history.push('/register/' + type + '/' + item.name)}>
              <TableCell component="th" scope="row">
                {item.name}
              </TableCell>
              <TableCell align="left">{item.description}</TableCell>
              <TableCell align="left">{item.example}</TableCell>
              <TableCell align="left">{item.write_access ? 'R/W' : 'R'}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </Paper>
  </div>
)

export default withRouter(RegisterItems);