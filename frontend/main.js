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

  win.loadURL(
    process.env.ELECTRON_START_URL ||
      `file://${path.join(__dirname, 'build/index.html')}`
  );
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

// IPC handler to run Python script
ipcMain.handle('run-python', async (event, args) => {
  let scriptPath = args[0];
  if (scriptPath === 'video_main.py' || scriptPath === '3Dviewer/main.py') {
    scriptPath = path.resolve(__dirname, '..', scriptPath);
  }
  const pyArgs = [scriptPath, ...args.slice(1)];
  return await new Promise((resolve, reject) => {
    const py = spawn('python3', pyArgs);

    let stdout = '';
    let stderr = '';

    py.stdout.on('data', (data) => { stdout += data.toString(); });
    py.stderr.on('data', (data) => { stderr += data.toString(); });

    py.on('close', (code) => {
      if (code === 0) resolve(stdout);
      else reject(stderr);
    });
  });
});

ipcMain.handle('select-folder', async () => {
  const result = await dialog.showOpenDialog({
    title: 'Select Data Folder',
    buttonLabel: 'Select Folder',
    defaultPath: app.getPath('home'),
    properties: ['openDirectory', 'dontAddToRecent']
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  return result.filePaths[0];
});

ipcMain.handle('select-zip', async () => {
  const result = await dialog.showOpenDialog({
    title: 'Select Data Zip File',
    buttonLabel: 'Select Zip',
    defaultPath: app.getPath('home'),
    properties: ['openFile', 'dontAddToRecent'],
    filters: [
      { name: 'Zip Files', extensions: ['zip'] }
    ]
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  return result.filePaths[0];
});

ipcMain.handle('unzip-to-temp', async (event, zipPath) => {
  if (!zipPath || !zipPath.endsWith('.zip')) {
    throw new Error('Selected file is not a .zip file.');
  }
  const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'metacam-'));
  // Use system unzip for robust extraction
  const unzipResult = spawnSync('unzip', [zipPath, '-d', tempDir], { encoding: 'utf-8' });
  if (unzipResult.error) {
    fs.rmSync(tempDir, { recursive: true, force: true });
    throw new Error('Failed to run unzip: ' + unzipResult.error.message);
  }
  if (unzipResult.status !== 0) {
    fs.rmSync(tempDir, { recursive: true, force: true });
    throw new Error('Unzip failed: ' + unzipResult.stderr);
  }
  return tempDir;
});