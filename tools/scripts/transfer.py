import sys
import json
import requests

from common.api import Api
from common.params import Params
from tools.lib.api import CommaApi as ToolsApi
from urllib.parse import urlparse

FILE_TRANSFER_ORDER = ["qlogs", "logs", "cameras", "dcameras", "qcameras"]

def transfer_route(route, tools_api, api, dongle_id):
    print("Begin transfer of route: " + route)

    files = tools_api.get('v1/route/' + route + '/files')

    for key in FILE_TRANSFER_ORDER:
        for url in files[key]:
            url_file = requests.get(url)
            dat = url_file.content

            print(f"Done downloading {route}. Prepairing upload...")

            _, _, time_str, segment_num, fn = urlparse(url).path.rsplit('/', maxsplit=4)
            segment_file = f'{time_str}--{segment_num}/{fn}'
            print(f"Getting upload URL for: {segment_file}")


            url_resp = api.get("v1.3/" + dongle_id + "/upload_url/", timeout=10, path=segment_file, access_token=api.get_token())
            url_resp_json = json.loads(url_resp.text)
            upload_url = url_resp_json['url']
            headers = url_resp_json['headers']

            print(f"Uploading: {segment_file}")
            upload_resp = requests.put(upload_url, data=dat, headers=headers, timeout=10)

            if upload_resp is not None and upload_resp.status_code in (200, 201):
                print(f"Finished uploading: {segment_file}")
            

def main(args):
    old_dongle_id = args[0]
    token = args[1]

    assert old_dongle_id, "Old device's dongle ID is required"
    assert token, "Comma API token is required"

    route = None if len(args) < 3 else args[2]

    tools_api = ToolsApi(token=token)

    params = Params()
    dongle_id = params.get("DongleId").decode('utf8')

    api = Api(dongle_id)

    if route:
        transfer_route(route, tools_api, api, dongle_id)
    else:
        # TODO find all old routes and transfer all of them 
        pass
    
        

if __name__ == "__main__":
    main(sys.argv[1:])