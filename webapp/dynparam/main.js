// Kết nối đến ROS
const ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.100:9090' // Thay bằng địa chỉ ROSBridge
  });
  
  // Lắng nghe topic parameter_updates
  const parameterUpdatesListener = new ROSLIB.Topic({
    ros: ros,
    name: '/agv_dynparam/parameter_updates', // Thay bằng tên topic của bạn
    messageType: 'dynamic_reconfigure/Config'
  });
  
  // Gọi dịch vụ để thay đổi tham số
  const reconfigureService = new ROSLIB.Service({
    ros: ros,
    name: '/agv_dynparam/set_parameters', // Thay bằng dịch vụ của bạn
    serviceType: 'dynamic_reconfigure/Reconfigure'
  });
  
  // Hàm lắng nghe và hiển thị tham số
  parameterUpdatesListener.subscribe((message) => {
    const paramsTable = document.getElementById('paramsTable').querySelector('tbody');
    paramsTable.innerHTML = ''; // Xóa danh sách cũ
  
    // Lặp qua các tham số kiểu số nguyên
    message.ints.forEach(param => {
      const row = createRow(param.name, 'int', param.value);
      paramsTable.appendChild(row);
    });
  
    // Lặp qua các tham số kiểu số thực
    message.doubles.forEach(param => {
      const row = createRow(param.name, 'double', param.value);
      paramsTable.appendChild(row);
    });
  
    // Lặp qua các tham số kiểu chuỗi
    message.strs.forEach(param => {
      const row = createRow(param.name, 'string', param.value);
      paramsTable.appendChild(row);
    });
  
    // Lặp qua các tham số kiểu boolean
    message.bools.forEach(param => {
      const row = createRow(param.name, 'bool', param.value);
      paramsTable.appendChild(row);
    });
  });
  
  // Hàm tạo một hàng trong bảng
  function createRow(name, type, value) {
    const row = document.createElement('tr');
    const valueCell = document.createElement('td');
  
    row.innerHTML = `
      <td>${name}</td>
      <td>${type}</td>
    `;
  
    if (type === 'bool') {
      valueCell.innerHTML = `<select>
        <option value="true" ${value ? 'selected' : ''}>True</option>
        <option value="false" ${!value ? 'selected' : ''}>False</option>
      </select>`;
    } else {
      valueCell.innerHTML = `<input type="${type === 'int' || type === 'double' ? 'number' : 'text'}" value="${value}">`;
    }
  
    row.appendChild(valueCell);
  
    const actionCell = document.createElement('td');
    const updateButton = document.createElement('button');
    updateButton.textContent = 'Update';
    updateButton.onclick = () => {
      const newValue = valueCell.querySelector('input, select').value;
      updateParameter(name, type, newValue);
    };
    actionCell.appendChild(updateButton);
    row.appendChild(actionCell);
  
    return row;
  }
  
  // Hàm cập nhật tham số
  function updateParameter(name, type, value) {
    const request = new ROSLIB.ServiceRequest({
      config: {
        [type === 'int' ? 'ints' : type === 'double' ? 'doubles' : type === 'string' ? 'strs' : 'bools']: [
          {
            name: name,
            value: type === 'bool' ? (value === 'true') : (type === 'int' ? parseInt(value) : type === 'double' ? parseFloat(value) : value)
          }
        ]
      }
    });
  
    reconfigureService.callService(request, (result) => {
      console.log(`Updated ${name} to ${value}:`, result);
    });
  }
  