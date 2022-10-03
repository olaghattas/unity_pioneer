import requests
import zipfile
import os


def main():
    URL = "https://universitysystemnh-my.sharepoint.com/:u:/g/personal/pac48_usnh_edu/ESRJjD6m6GFFuFTXJMntsawBqHCD9x1y2ILIgI6zN5Z09A?download=1"
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

    os.system('chmod +x ' + os.path.join(install_path, 'sim', 'pioneerSimulation.x86_64'))
    os.system(os.path.join(install_path, 'sim', 'pioneerSimulation.x86_64  -batchmode'))


if __name__ == '__main__':
    main()
