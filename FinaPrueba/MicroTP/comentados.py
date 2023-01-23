'''i=0
    countL=0
    countR=0
    while i<=150000:
        i=i+1
        if GPIO.event_detected(ENCODERL):
            countL=contador(countL)
        if GPIO.event_detected(ENCODERR):
            countR=contador(countR)
    print("ContadorL:",countL)
    print("ContadorR:",countR)'''
    '''with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        method_thread = {
            executor.submit(countEncoder, ENCODERL):
            "process completed encoderL",
            executor.submit(countEncoder, ENCODERR):
            "process completed encoderR",
        }
    for future in concurrent.futures.as_completed(method_thread):
        etiqueta = method_thread[future]
        try:
            if etiqueta == "process completed encoderL":
                tiempoL=future.result()
            if etiqueta == "process completed encoderR":
                tiempoR=future.result()
        except Exception as exc:
            print('%r se a generado una exception: %s' % (final, exc))'''