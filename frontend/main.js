const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const path = require('path');
const { spawn, spawnSync } = require('child_process');
const fs = require('fs');
const os = require('os');

function createWindow () {
  const win = new BrowserWindow({
    width: 900,
    height: 700,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    }
  });

  // Always load the React dev server for development
  win.loadURL('http://localhost:3000');
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

let lastTempDir = null;

// One-click workflow: select zip, unzip, preprocess, view
ipcMain.handle('process-zip-and-view', async (event) => {
  const webContents = event.sender;

  // 1. Select zip file
  const result = await dialog.showOpenDialog({
    title: 'Select Data Zip File',
    buttonLabel: 'Select Zip',
    defaultPath: app.getPath('home'),
    properties: ['openFile', 'dontAddToRecent'],
    filters: [
      { name: 'Zip Files', extensions: ['zip'] }
    ]
  });
  if (result.canceled || result.filePaths.length === 0) throw new Error('No zip file selected.');
  const zipPath = result.filePaths[0];

  webContents.send('progress-update', { step: 1, message: 'Unzipping data...' });

  // 1. Get root folder name from zip
  const zipList = spawnSync('unzip', ['-l', zipPath], { encoding: 'utf-8' });
  const match = zipList.stdout.match(/^\s+\d+\s+\d{4}-\d{2}-\d{2} \d{2}:\d{2}\s+([^/]+)\//m);
  if (!match) throw new Error('Could not determine root folder in zip');
  const zipRootFolder = match[1];
  const tempDataDir = path.join('/tmp', zipRootFolder);

  // Remove the extraction directory if it exists
  if (fs.existsSync(tempDataDir)) {
    fs.rmSync(tempDataDir, { recursive: true, force: true });
  }

  // 2. Unzip to /tmp
  const unzipResult = spawnSync('unzip', [zipPath, '-d', '/tmp'], { encoding: 'utf-8' });
  if (unzipResult.error) {
    throw new Error('Failed to run unzip: ' + unzipResult.error.message);
  }
  if (unzipResult.status !== 0) {
    throw new Error('Unzip failed: ' + unzipResult.stderr);
  }

  webContents.send('progress-update', { step: 2, message: 'Preprocessing data...' });

  // 3. Pass /tmp/<zip-root-folder> to pp.sh
  await new Promise((resolve, reject) => {
    const pp = spawn('bash', ['pp.sh', tempDataDir], { cwd: path.resolve(__dirname, '..') });

    pp.stdout.on('data', (data) => {
      process.stdout.write(data.toString());
    });
    pp.stderr.on('data', (data) => {
      process.stderr.write(data.toString());
    });

    let stdout = '', stderr = '';
    pp.stdout.on('data', (data) => { stdout += data.toString(); });
    pp.stderr.on('data', (data) => { stderr += data.toString(); });
    pp.on('close', (code) => {
      if (code === 0) resolve(stdout);
      else reject(stderr || stdout);
    });
  });

  webContents.send('progress-update', { step: 3, message: 'Preprocessing complete. Ready to view!' });

  // DO NOT launch the viewer here!
  // The viewer will be launched when the user selects a point cloud in the UI.

  // Just return success
  return "Preprocessing complete";
});

ipcMain.handle('list-output-dirs', async () => {
  const outputDir = path.resolve(__dirname, '..', 'fisheye_ws', 'output');
  if (!fs.existsSync(outputDir)) return [];
  return fs.readdirSync(outputDir)
    .filter(f => fs.statSync(path.join(outputDir, f)).isDirectory())
    .map(f => path.join(outputDir, f));
});

ipcMain.handle('run-viewer', async (event, dir) => {
  console.log('Launching 3D viewer for:', dir);
  return await new Promise((resolve, reject) => {
    const py = spawn('python3', ['/home/user/metacam-edu-reader/main.py', dir], { cwd: path.resolve(__dirname, '..') });
    let stdout = '', stderr = '';
    py.stdout.on('data', (data) => { stdout += data.toString(); });
    py.stderr.on('data', (data) => { stderr += data.toString(); });
    py.on('close', (code) => {
      if (code === 0) resolve(stdout);
      else reject(stderr || stdout);
    });
  });
});

ipcMain.handle('select-folder', async () => {
  return await dialog.showOpenDialog({ properties: ['openDirectory'] });
});
ipcMain.handle('select-zip', async () => {
  return await dialog.showOpenDialog({ filters: [{ name: 'Zip Files', extensions: ['zip'] }] });
});

const pythonCmd = 'python3'; // or your conda python path if needed

ipcMain.handle('check-rerun', async () => {
  const code = `
try:
    import rerun
    print("OK")
except ImportError:
    print("NO")
`;
  const result = spawnSync(pythonCmd, ['-c', code]);
  const output = result.stdout.toString().trim();
  return output === "OK";
});

const POINTCLOUD_DIR = path.resolve(__dirname, '..', 'output', 'pointcloud');

ipcMain.handle('list-pointcloud-files', async () => {
  if (!fs.existsSync(POINTCLOUD_DIR)) return [];
  return fs.readdirSync(POINTCLOUD_DIR)
    .filter(f => f.endsWith('.pcd'))
    .map(f => path.join(POINTCLOUD_DIR, f));
});

ipcMain.handle('run-viewer-pointcloud', async (event, pcdPath) => {
  return await new Promise((resolve, reject) => {
    const py = spawn('python3', [
      '/home/user/metacam-edu-reader/main.py',
      '--pointcloud', pcdPath
    ], { cwd: path.resolve(__dirname, '..') });
    let stdout = '', stderr = '';
    py.stdout.on('data', (data) => { stdout += data.toString(); });
    py.stderr.on('data', (data) => { stderr += data.toString(); });
    py.on('close', (code) => {
      if (code === 0) resolve(stdout);
      else reject(stderr || stdout);
    });
  });
});

// For "view all" option:
ipcMain.handle('run-viewer-all-pointclouds', async () => {
  return await new Promise((resolve, reject) => {
    const py = spawn('python3', [
      '/home/user/metacam-edu-reader/main.py',
      '--all-pointclouds'
    ], { cwd: path.resolve(__dirname, '..') });
    let stdout = '', stderr = '';
    py.stdout.on('data', (data) => { stdout += data.toString(); });
    py.stderr.on('data', (data) => { stderr += data.toString(); });
    py.on('close', (code) => {
      if (code === 0) resolve(stdout);
      else reject(stderr || stdout);
    });
  });
});