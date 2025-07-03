const { contextBridge, ipcRenderer, shell } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  invoke: (...args) => ipcRenderer.invoke(...args),
  onProgressUpdate: (callback) => {
    ipcRenderer.on('progress-update', callback);
    return () => ipcRenderer.removeListener('progress-update', callback);
  },
  listOutputDirs: () => ipcRenderer.invoke('list-output-dirs'),
  runViewer: (dir) => ipcRenderer.invoke('run-viewer', dir),
  selectFolder: () => ipcRenderer.invoke('select-folder'),
  selectZip: () => ipcRenderer.invoke('select-zip'),
  openPath: (path) => shell.openPath(path),
  listPointcloudFiles: () => ipcRenderer.invoke('list-pointcloud-files'),
  runViewerPointcloud: (pcdPath) => ipcRenderer.invoke('run-viewer-pointcloud', pcdPath),
  runViewerAllPointclouds: () => ipcRenderer.invoke('run-viewer-all-pointclouds'),
});