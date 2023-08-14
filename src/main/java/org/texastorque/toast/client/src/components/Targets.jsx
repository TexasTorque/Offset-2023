import { Table } from "react-bootstrap";

const Targets = () => {
  return (
    <div>
      <h1>Targets</h1>
      <Table bordered hover variant="dark" className="rsp-table">
        <tbody>
          <tr>
            <td>fx: 123.431</td>
            <td>cx: 7372.432</td>
          </tr>
          <tr>
            <td>fy: 3873893.34</td>
            <td>cy: 47387332.</td>
          </tr>
        </tbody>
      </Table>
    </div>
  );
};

export default Targets;
