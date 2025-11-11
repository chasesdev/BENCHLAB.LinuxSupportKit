import requests

def stream(url='http://127.0.0.1:8080/stream'):
    with requests.get(url, stream=True) as r:
        r.raise_for_status()
        for line in r.iter_lines():
            if not line:
                continue
            print(line.decode())

if __name__ == '__main__':
    stream()
