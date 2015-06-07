#written by Don McLaine
import binascii

# return a dictionary with parsed data
def get_node_data(port):
    data = {}
    data['start_delimiter'] = port.read()
    if data['start_delimiter'] == b'~':                # 0x7f is b'~'
        data['bstr_length']     = port.read(2)
        data['length']          = int.from_bytes(data['bstr_length'], 'big')
        data['the_rest']        = port.read(data['length'] + 1)
        data['whole_packet']    = data['start_delimiter'] + data['bstr_length'] + data['the_rest']
        data['checksum']        = data['the_rest'][-1:]
        data['frame_type']      = data['the_rest'][0]    # this gives us an int
        if data['frame_type'] == 0x90:
            data['address_64']      = int.from_bytes(data['the_rest'][1:9], 'big')
            data['address_16']      = int.from_bytes(data['the_rest'][9:11], 'big')
            data['receive_options'] = data['the_rest'][11:12]
            data['contents']        = data['the_rest'][12:-1]
        elif data['frame_type'] == 0x81:
            data['address_16']      = int.from_bytes(data['the_rest'][1:3], 'big')
            data['RSSI']            = int.from_bytes(data['the_rest'][3:4], 'big')
            data['receive_options'] = data['the_rest'][4:5]
            data['contents']        = data['the_rest'][5:-1]
        else:
            raise Exception("Unhandled frame type")
        return data
    elif data['start_delimiter'] == b'':
        raise Exception("timeout")
    else:
        raise Exception("not a valid start delimiter:", binascii.hexlify(data['start_delimiter']))
