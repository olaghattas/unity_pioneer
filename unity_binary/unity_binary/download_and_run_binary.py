import requests
import zipfile
import os


def main():
    URL = "https://universitysystemnh-my.sharepoint.com/:u:/g/personal/mb1215_usnh_edu/EVM_16qj-QJDsMFiaw6DGYYBQEtt6hvDj211F_HHqjruzg?download=1"
    home_path = os.path.expanduser('~')
    install_path = os.path.join(home_path, 'unity_binary')
    installed = os.path.exists(install_path)

    if not installed:
        with requests.get(URL, stream=True) as r:
            print(f'downloading file: total size: {len(r.content)}')
            downloaded = 0
            with open("sim.zip", 'wb') as f:
                for chunk in r.iter_content(chunk_size=1024 * 64 * 16):
                    # writing one chunk at a time to pdf file
                    if chunk:
                        f.write(chunk)
                        downloaded += 1024 * 64 * 16
                        print(f'downloading [% {100 * (downloaded / len(r.content))}]')

        print('download complete')

        with zipfile.ZipFile('sim.zip', 'r') as zip_ref:
            zip_ref.extractall(install_path)

    os.system('chmod +x ' + os.path.join(install_path, 'sim', 'smart_home_unity_binary.x86_64'))
    os.system(os.path.join(install_path, 'sim', 'smart_home_unity_binary.x86_64'))


if __name__ == '__main__':
    main()
