import sys

MAX_PKT_SIZ = 20
DELIM_WORD = 'READ:'
RESP_CODE = 0x20


def cnt_nb_cmd_char(cmd_str):
    cnt = 0
    for c in cmd_str:
        if c == ',':
            cnt += 1
    return cnt


def build_cur_frame(payload_bytes, last_frame, is_cmd):
    nb_var_bytes = len(payload_bytes)
    cmd_name_asccii_str_lst = [str(hex(ord(c))) for c in payload_bytes]
    cmd_str = '{ 0x10, '
    if is_cmd is True:
        cmd_str += '0x00, '
    else:
        cmd_str += '0x01, '
    cmd_str += '0x0a, '
    if last_frame is True:
        cmd_str += str(hex(nb_var_bytes + 1)) + ', '  # + 1 for \n at end
    else:
        if is_cmd is True:
            cmd_str += str(hex(nb_var_bytes | 0x80)) + ', '  # MSB = 1 to indicate more frames incoming
        else:
            cmd_str += str(hex(nb_var_bytes)) + ', '
    nb_fix_bytes = cnt_nb_cmd_char(cmd_str)
    for s in cmd_name_asccii_str_lst:
        cmd_str += s + ', '
    if last_frame is True:
        cmd_str += '0x0a, '
        nb_fix_bytes += 1
    nb_cmd_char = cnt_nb_cmd_char(cmd_str)
    if nb_cmd_char > MAX_PKT_SIZ:
        print('wrong arg passed to this function')
        exit(0)
    elif nb_cmd_char < MAX_PKT_SIZ:
        for i in range(MAX_PKT_SIZ - (nb_fix_bytes + nb_var_bytes)):
            cmd_str += '0xff, '
    cmd_str += '] '
    return cmd_str


def cmd_to_str(cmd_name, is_cmd):
    nb_fix_bytes = 5  # cmd (1) + cmd code(2) + nbBytesInCurPayload(1) + ... + '\n' (1)
    nb_var_bytes = len(cmd_name)
    cmd_str = ''
    cmd_nam_lst = list()
    if nb_fix_bytes + nb_var_bytes > MAX_PKT_SIZ:
        split_val = MAX_PKT_SIZ - (nb_fix_bytes - 1)
        cmd_nam_lst = [cmd_name[i:i+split_val] for i in range(0, len(cmd_name), split_val)]
        for i, cmd_nam in enumerate(cmd_nam_lst):
            if i == len(cmd_nam_lst) - 1:
                cmd_str += build_cur_frame(cmd_nam, True, is_cmd)
            else:
                cmd_str += build_cur_frame(cmd_nam, False, is_cmd) + '\n'
    else:
        cmd_str += build_cur_frame(cmd_name, True, is_cmd)
    cmd_str += '\n'
    return cmd_str


def parse_n_format(op_str):
    op_val = ''
    op_str_lines = op_str.splitlines()
    for line in op_str_lines:
        substr_pos = line.find(DELIM_WORD)
        if substr_pos != -1:
            op_substr = line[substr_pos + len(DELIM_WORD):]
            op_words = op_substr.split()
            for i, op_word in enumerate(op_words):
                if (i == 0) or (i != 0 and op_word != '0xFF') :
                    try:
                        op_val += str(hex(int(op_word, 16))) + ', '
                    except Exception:
                        pass
    return op_val


def resp_to_str(op_str):
    op_val_str = parse_n_format(op_str)
    op_val_int_lst = list()
    for op_val in op_val_str.split(','):
        try:
            op_val_int_lst.append(int(op_val, 16))
        except Exception:
            pass
    resp_str = ''
    if op_val_int_lst[0] == RESP_CODE:
        for val in op_val_int_lst[4:]:
            resp_str += str(chr(val))
        if op_val_int_lst[3] == 0x90:
            resp_str += '\nsome more packets remaining, execute one more read cycle using cmd: ' \
                        '{0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0xff' \
                        '0xff 0xff 0xff 0xff]\n'
        else:
            resp_str += '\nend of response\n'
    return resp_str


def test():
    print(cmd_to_str('AT+BLEGETADDR'))
    resp_str = resp_to_str("""[05:52:34:389] /CS ENABLED␍␊
[05:52:34:389] WRITE: 0x10 READ: 0x20 ␍␊
[05:52:34:389] WRITE: 0x00 READ: 0x00 ␍␊
[05:52:34:389] WRITE: 0x0A READ: 0x0A ␍␊
[05:52:34:389] WRITE: 0x04 READ: 0x90 ␍␊
[05:52:34:389] WRITE: 0x41 READ: 0x42 ␍␊
[05:52:34:389] WRITE: 0x54 READ: 0x4C ␍␊
[05:52:34:389] WRITE: 0x49 READ: 0x45 ␍␊
[05:52:34:389] WRITE: 0x0A READ: 0x53 ␍␊
[05:52:34:389] READ: 0x50 0x49 0x46 0x52 0x49 0x45 0x4E 0x44 0x0D 0x0A 0x6E 0x52 ␍␊
[05:52:34:389] /CS DISABLED␍␊
""")
    print(resp_str)
    resp_str = resp_to_str("""␍␊
    [05:52:34:389] SPI>[r:20]␍␊
    [05:52:36:322] /CS ENABLED␍␊
    [05:52:36:322] READ: 0x20 0x00 0x0A 0x90 0x46 0x35 0x31 0x38 0x32 0x32 0x20 0x51 0x46 0x41 0x43 0x41 0x30 0x30 0x0D 0x0A ␍␊
    [05:52:36:322] /CS DISABLED␍␊
    """)
    print(resp_str)
    resp_str = resp_to_str("""␍␊
    [05:52:36:322] SPI>[r:20]␍␊
    [05:52:37:361] /CS ENABLED␍␊
    [05:52:37:361] READ: 0x20 0x00 0x0A 0x90 0x42 0x4C 0x45 0x53 0x50 0x49 0x46 0x52 0x49 0x45 0x4E 0x44 0x0D 0x0A 0x6E 0x52 ␍␊
    [05:52:37:382] /CS DISABLED␍␊
    """)
    print(resp_str)
    resp_str = resp_to_str("""␍␊
        [05:52:37:382] SPI>[r:20]␍␊
        [05:52:38:097] /CS ENABLED␍␊
        [05:52:38:097] READ: 0x20 0x00 0x0A 0x90 0x46 0x35 0x31 0x38 0x32 0x32 0x20 0x51 0x46 0x41 0x43 0x41 0x30 0x30 0x0D 0x0A ␍␊
        [05:52:38:097] /CS DISABLED␍␊
        """)
    print(resp_str)
    resp_str = resp_to_str("""␍␊
        [05:52:38:097] SPI>[r:20]␍␊
        [05:52:38:816] /CS ENABLED␍␊
        [05:52:38:816] READ: 0x20 0x00 0x0A 0x90 0x37 0x35 0x43 0x31 0x46 0x38 0x46 0x39 0x35 0x33 0x44 0x33 0x34 0x42 0x35 0x44 ␍␊
        [05:52:38:816] /CS DISABLED␍␊""")
    print(resp_str)
    resp_str = resp_to_str("""␍␊
            [05:52:38:816] SPI>[r:20]␍␊
            [05:52:39:473] /CS ENABLED␍␊
            [05:52:39:473] READ: 0x20 0x00 0x0A 0x90 0x0D 0x0A 0x30 0x2E 0x38 0x2E 0x31 0x0D 0x0A 0x30 0x2E 0x38 0x2E 0x31 0x0D 0x0A ␍␊
            [05:52:39:473] /CS DISABLED␍␊""")
    print(resp_str)
    resp_str = resp_to_str("""␍␊
            [05:52:39:473] SPI>[r:20]␍␊
            [05:52:40:161] /CS ENABLED␍␊
            [05:52:40:161] READ: 0x20 0x00 0x0A 0x90 0x41 0x70 0x72 0x20 0x31 0x30 0x20 0x32 0x30 0x31 0x39 0x0D 0x0A 0x53 0x31 0x31 ␍␊
            [05:52:40:182] /CS DISABLED␍␊""")
    print(resp_str)
    resp_str = resp_to_str("""␍␊
            [05:52:40:182] SPI>[r:20]␍␊
            [05:52:41:912] /CS ENABLED␍␊
            [05:52:41:912] READ: 0x20 0x00 0x0A 0x90 0x30 0x20 0x38 0x2E 0x30 0x2E 0x30 0x2C 0x20 0x30 0x2E 0x32 0x0D 0x0A 0x4F 0x4B ␍␊
            [05:52:41:912] /CS DISABLED␍␊""")
    print(resp_str)
    resp_str = resp_to_str("""␍␊
            [05:52:41:912] SPI>[r:20]␍␊
            [05:52:42:630] /CS ENABLED␍␊
            [05:52:42:630] READ: 0x20 0x00 0x0A 0x02 0x0D 0x0A 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF ␍␊
            [05:52:42:630] /CS DISABLED␍␊""")
    print(resp_str)


def main():
    val = input('0:test\n1:resp\nelse:req\n')
    if val == '0':
        test()
    elif val == '1':
        print('paste your string n use ctrl-D to terminate')
        resp_str = sys.stdin.read()
        print(resp_to_str(resp_str))
    else:
        is_cmd = input('0:Cmd\nelse:Dat\n')
        if is_cmd == '0':
            cmd_str = input()
            pkt_str = cmd_to_str(cmd_str, True)
        else:
            tx_rx_cmd = input('0:rx\nelse:tx\n')
            if tx_rx_cmd == '0':
                pkt_str = '{0x10, 0x02, 0x0a, 0x00, 0xff, 0xff, 0xff, 0xff,' \
                          ' 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,]'
            else:
                cmd_str = input()
                pkt_str = cmd_to_str(cmd_str, False)
        print(pkt_str)


if __name__ == '__main__':
    main()

