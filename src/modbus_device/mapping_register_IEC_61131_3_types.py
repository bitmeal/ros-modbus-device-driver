
class RegisterTypeMapping:
    mapping = {
        "BYTE": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_byte(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_byte(value),
            "size": lambda **kwargs: 0.5
        },
        "WORD":     {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_word(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_word(value),
            "size": lambda **kwargs: 1
        },
        "DWORD": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_dword(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_dword(value),
            "size": lambda **kwargs: 2
        },
        "LWORD": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_lword(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_lword(value),
            "size": lambda **kwargs: 4
        },
        "SINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_8bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_8bit_int(value),
            "size": lambda **kwargs: 0.5
        },
        "INT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_16bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_16bit_int(value),
            "size": lambda **kwargs: 1
        },
        "DINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_32bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_32bit_int(value),
            "size": lambda **kwargs: 2
        },
        "LINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_64bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_64bit_int(value),
            "size": lambda **kwargs: 4
        },
        "USINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_8bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_8bit_uint(value),
            "size": lambda **kwargs: 0.5
        },
        "UINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_16bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_16bit_uint(value),
            "size": lambda **kwargs: 1
        },
        "UDINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_32bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_32bit_uint(value),
            "size": lambda **kwargs: 2
        },
        "ULINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_64bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_64bit_uint(value),
            "size": lambda **kwargs: 4
        },
        "REAL": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_32bit_float(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_32bit_float(value),
            "size": lambda **kwargs: 2
        },
        "LREAL": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_64bit_float(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_64bit_float(value),
            "size": lambda **kwargs: 4
        },
        "CHAR": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_string(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_string(value),
            "size": lambda **kwargs: 0.5
        },
        "STRING": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_string(RegisterTypeMapping.fix_string_alignment(**kwargs)['length'], **kwargs),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_string(value, **kwargs),
            "size": lambda **kwargs: RegisterTypeMapping.fix_string_alignment(**kwargs)['length'] * 0.5
        },
        # "WCHAR": {
        #     "decoder": lambda **kwargs: lambda decoder: decoder.decode_string(),
        #     "size": lambda **kwargs: 0.5
        # },
        # "WSTRING": {
        #     "decoder": lambda **kwargs: lambda decoder: decoder.decode_wstring(kwargs['length'], **kwargs),
        #     "size": lambda **kwargs: kwargs['length']
        # }
    }

    @staticmethod
    def fix_string_alignment(**kwargs):
        if(not kwargs.get('ignore_byteorder', False)):
            # kwargs = {**kwargs, 'length': kwargs['length'] + kwargs['length']%2} #PYTHON3
            kwargs.update({'length': kwargs['length'] + kwargs['length']%2})
        return kwargs

    @staticmethod
    def get_size(type, **kwargs):
        return RegisterTypeMapping.mapping[type.upper()]['size'](**kwargs)
    
    @staticmethod
    def get_decode_fun(type, **kwargs):
        return RegisterTypeMapping.mapping[type.upper()]['decoder'](**kwargs)

    @staticmethod
    def get_encode_fun(type, **kwargs):
        return RegisterTypeMapping.mapping[type.upper()]['encoder'](**kwargs)

