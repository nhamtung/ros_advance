// Kết nối ROS
const ros = new ROSLIB.Ros({
  url: 'ws://192.168.1.100:9090' // ROSBridge URL
});

// Lắng nghe các topic cần thiết
const parameterUpdatesListener = new ROSLIB.Topic({
  ros: ros,
  name: '/agv_dynparam/parameter_updates',
  messageType: 'dynamic_reconfigure/Config'
});

const parameterDescriptionsListener = new ROSLIB.Topic({
  ros: ros,
  name: '/agv_dynparam/parameter_descriptions',
  messageType: 'dynamic_reconfigure/ParameterDescriptions'
});

const reconfigureService = new ROSLIB.Service({
  ros: ros,
  name: '/agv_dynparam/set_parameters',
  serviceType: 'dynamic_reconfigure/Reconfigure'
});

// Tạo dịch vụ để lưu tham số
const saveParamsService = new ROSLIB.Service({
  ros: ros,
  name: '/agv_dynparam/save_dynparam_runtime',
  serviceType: 'std_srvs/Trigger'
});

let parameterDescriptions = {};

// Nhận thông tin mô tả tham số
parameterDescriptionsListener.subscribe((message) => {
  parameterDescriptions = message;
});

// Lắng nghe cập nhật tham số
parameterUpdatesListener.subscribe((message) => {
  updateParametersTable(message);
});

// Cập nhật bảng
function updateParametersTable(config) {
  const tableBody = document.getElementById('paramsTable').querySelector('tbody');
  tableBody.innerHTML = ''; // Xóa bảng cũ
  let rowNumber = 1;

  // Hiển thị từng loại tham số
  ['ints', 'doubles', 'strs', 'bools'].forEach(type => {
    config[type].forEach(param => {
      const row = createRow(rowNumber++, param.name, type.slice(0, -1), param.value);
      tableBody.appendChild(row);
    });
  });
}

// Tạo hàng mới
function createRow(rowNumber, name, type, value) {
  const row = document.createElement('tr');

  row.innerHTML = `
    <td>${rowNumber}</td>
    <td>${name}</td>
    <td>${type}</td>
    <td id="min-${name}">${getMinValue(name)}</td>
    <td>
      ${type === 'bool' ? `
        <select>
          <option value="true" ${value ? 'selected' : ''}>True</option>
          <option value="false" ${!value ? 'selected' : ''}>False</option>
        </select>
      ` : `
        <input type="${type === 'int' || type === 'double' ? 'number' : 'text'}" value="${value}">
      `}
    </td>
    <td id="max-${name}">${getMaxValue(name)}</td>
    <td id="desc-${name}">${getDescription(name)}</td>
    <td>
      <button onclick="updateParameter('${name}', '${type}', this)">Update</button>
    </td>
  `;

  return row;
}

// Lấy min/max/description từ parameterDescriptions
function getMinValue(name) {
  const param = parameterDescriptions.params?.find(p => p.name === name);
  return param ? param.min : 'N/A';
}

function getMaxValue(name) {
  const param = parameterDescriptions.params?.find(p => p.name === name);
  return param ? param.max : 'N/A';
}

function getDescription(name) {
  const param = parameterDescriptions.params?.find(p => p.name === name);
  return param ? param.description : 'N/A';
}

// Cập nhật tham số
function updateParameter(name, type, button) {
  const row = button.closest('tr');
  const input = row.querySelector('input, select');
  const value = input.type === 'number' ? parseFloat(input.value) : input.value === 'true';

  const request = new ROSLIB.ServiceRequest({
    config: {
      [type + 's']: [{ name, value }]
    }
  });

  reconfigureService.callService(request, (result) => {
    console.log(`Updated ${name} to ${value}`, result);
  });
}


// Lắng nghe sự kiện click nút Save
document.getElementById('saveButton').addEventListener('click', () => {
  saveParamsService.callService(new ROSLIB.ServiceRequest(), (result) => {
    if (result.success) {
      alert('Parameters saved successfully!');
    } else {
      alert(`Failed to save parameters: ${result.message}`);
    }
  });
});