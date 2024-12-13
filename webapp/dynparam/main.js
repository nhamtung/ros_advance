// Kết nối đến ROS
const ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.100:9090' // Thay bằng địa chỉ của ROSBridge
  });
  
  // Lắng nghe topic parameter_updates
  const parameterUpdatesListener = new ROSLIB.Topic({
    ros: ros,
    name: '/agv_dynparam/parameter_updates',
    messageType: 'dynamic_reconfigure/Config'
  });
  
  // Lắng nghe topic parameter_descriptions để lấy thông tin về min/max và description
  const parameterDescriptionsListener = new ROSLIB.Topic({
    ros: ros,
    name: '/agv_dynparam/parameter_descriptions',
    messageType: 'dynamic_reconfigure/ParameterDescriptions'
  });
  
  // Gọi dịch vụ để thay đổi tham số
  const reconfigureService = new ROSLIB.Service({
    ros: ros,
    name: '/agv_dynparam/set_parameters',
    serviceType: 'dynamic_reconfigure/Reconfigure'
  });
  
  // Để lưu trữ thông tin min/max và description của các tham số
  let parameterDescriptions = {};
  
  // Lắng nghe và lưu thông tin parameter_descriptions
  parameterDescriptionsListener.subscribe((message) => {
    parameterDescriptions = message;
    updateParametersTable();  // Sau khi nhận được thông tin descriptions, cập nhật bảng
  });
  
  // Hàm lắng nghe và hiển thị tham số
  parameterUpdatesListener.subscribe((message) => {
    const paramsTable = document.getElementById('paramsTable').querySelector('tbody');
    paramsTable.innerHTML = ''; // Xóa danh sách cũ
    let rowNumber = 1;
  
    // Lặp qua các tham số kiểu số nguyên
    message.ints.forEach(param => {
      const row = createRow(rowNumber, param.name, 'int', param.value);
      updateMinMax(param.name, row);  // Cập nhật min/max cho tham số
      updateDescription(param.name, row); // Cập nhật description cho tham số
      paramsTable.appendChild(row);
      rowNumber++;  // Tăng số thứ tự
    });
  
    // Lặp qua các tham số kiểu số thực
    message.doubles.forEach(param => {
      const row = createRow(rowNumber, param.name, 'double', param.value);
      updateMinMax(param.name, row);  // Cập nhật min/max cho tham số
      updateDescription(param.name, row); // Cập nhật description cho tham số
      paramsTable.appendChild(row);
      rowNumber++;  // Tăng số thứ tự
    });
  
    // Lặp qua các tham số kiểu chuỗi
    message.strs.forEach(param => {
      const row = createRow(rowNumber, param.name, 'string', param.value);
      updateMinMax(param.name, row);  // Cập nhật min/max cho tham số
      updateDescription(param.name, row); // Cập nhật description cho tham số
      paramsTable.appendChild(row);
      rowNumber++;  // Tăng số thứ tự
    });
  
    // Lặp qua các tham số kiểu boolean
    message.bools.forEach(param => {
      const row = createRow(rowNumber, param.name, 'bool', param.value);
      updateMinMax(param.name, row);  // Cập nhật min/max cho tham số
      updateDescription(param.name, row); // Cập nhật description cho tham số
      paramsTable.appendChild(row);
      rowNumber++;  // Tăng số thứ tự
    });
  });
  
  // Hàm tạo một hàng trong bảng
  function createRow(rowNumber, name, type, value) {
    const row = document.createElement('tr');
    const valueCell = document.createElement('td');
  
    row.innerHTML = `
      <td>${rowNumber}</td> <!-- Cột số thứ tự -->
      <td>${name}</td>
      <td>${type}</td>
      <td id="min-${name}"></td>
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
    const maxCell = document.createElement('td');
    maxCell.innerHTML = `
      <td id="max-${name}"></td>
    `;
    row.appendChild(maxCell);

    const descriptionCell = document.createElement('td');
    descriptionCell.innerHTML = `
      <td id="desc-${name}"></td>  <!-- Cột Description -->
    `;
    row.appendChild(descriptionCell);
  
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
  
  // Hàm cập nhật min/max cho tham số
  function updateMinMax(paramName, row) {
    if (parameterDescriptions && parameterDescriptions.params) {
      // Tìm kiếm mô tả tham số trong danh sách params
      const paramDescription = parameterDescriptions.params.find(param => param.name === paramName);
      
      if (paramDescription) {
        // Lấy các giá trị min và max
        const min = paramDescription.min;
        const max = paramDescription.max;
        
        // Cập nhật giá trị min/max vào bảng
        const minCell = row.querySelector(`#min-${paramName}`);
        const maxCell = row.querySelector(`#max-${paramName}`);
        
        minCell.innerText = min !== undefined ? min : 'N/A';
        maxCell.innerText = max !== undefined ? max : 'N/A';
      }
    }
  }
  
  // Hàm cập nhật description cho tham số
  function updateDescription(paramName, row) {
    if (parameterDescriptions && parameterDescriptions.params) {
      const paramDescription = parameterDescriptions.params.find(param => param.name === paramName);
      const descriptionCell = row.querySelector(`#desc-${paramName}`);
      
      if (paramDescription && paramDescription.description) {
        descriptionCell.innerText = paramDescription.description;
      } else {
        descriptionCell.innerText = 'N/A'; // Nếu không có description
      }
    }
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
  
  // Hàm thêm khả năng thay đổi độ rộng của các cột độc lập
  document.querySelectorAll('.resizable').forEach(th => {
    const resizer = document.createElement('div');
    resizer.classList.add('resizer');
    th.appendChild(resizer);
  
    let startX, startWidth;
  
    resizer.addEventListener('mousedown', (e) => {
      startX = e.clientX;
      startWidth = th.offsetWidth;
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', () => {
        document.removeEventListener('mousemove', handleMouseMove);
      });
    });
  
    function handleMouseMove(e) {
      const diff = e.clientX - startX;
      th.style.width = `${startWidth + diff}px`;
    }
  });
  