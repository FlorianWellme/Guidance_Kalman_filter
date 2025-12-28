import serial

if __name__ == "__main__":
    # Adapte le port à ton cas
    port = "/dev/cu.usbmodem142301"
    baud = 115200

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connexion ouverte sur {port} à {baud} baud.\n")
        
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("Reçu :", line)

    except Exception as e:
        print("Erreur :", e)
