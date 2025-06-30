const { contextBridge, ipcRenderer } = require('electron');
contextBridge.exposeInMainWorld('electronAPI', {
  runPython: (args) => ipcRenderer.invoke('run-python', args),
  selectFolder: () => ipcRenderer.invoke('select-folder'),
  selectZip: () => ipcRenderer.invoke('select-zip'),
  unzipToTemp: (zipPath) => ipcRenderer.invoke('unzip-to-temp', zipPath)
});