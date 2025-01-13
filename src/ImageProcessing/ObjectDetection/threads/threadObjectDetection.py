import cv2
from ultralytics import YOLO

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import(ObjectDetection)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
class threadObjectDetection(ThreadWithStop):
    """This thread handles ObjectDetection.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(threadObjectDetection, self).__init__()
        
        # Sender
        self.objectDetectionSender = messageHandlerSender(self.queuesList, ObjectDetection)
        self.subscribe()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass

    def run(self):
        while self._running:
            try:
                objects = self.main()
                self.objectDetectionSender.send(objects)
            except Exception as e:
                print(e)
    
    # def annotate_boxes(self, frame, results, model):
    #     for box in results.boxes:
    #         x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
    #         cls = int(box.cls[0])
    #         conf = box.conf[0].item()

    #         # Koristi model.model.names za naziv klase
    #         label = f"{model.model.names[cls]}: {conf:.2f}"

    #         #Crtanje bounding box-a
    #         cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    #         #Ispisivanje klase i poverenja na slici
    #         cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    #     return frame

    def main(self):
        # Učitaj model
        model = YOLO('yolov8n.pt')

        # Putanja do slike
        image_path = 'stop_test.jpg'  # Zameni sa putanjom do svoje slike

        # Učitaj sliku
        frame = cv2.imread(image_path)
        if frame is None:
            print("Greška pri učitavanju slike!")
            return
        
        frame = cv2.resize(frame, (640,480))

        # Pokreni model na slici
        results = model(frame)[0]

        # Annotiraj okvire na slici
        # frame = self.annotate_boxes(frame=frame, results=results, model=model)

        # Generiši labelu za detekcije (ispisuje naziv klase i poverenje)
        labels = [
            f"{model.model.names[int(cls)]} {conf:0.2f}"
            for cls, conf in zip(results.boxes.cls, results.boxes.conf)
        ]
        
        # Lista detektovanih objekata
        objects = []

        # Ispisivanje naziva klasa i poverenja na konzolu
        for label in labels:
            #print(label)
            objects.append(label)

        return objects
        # Prikaz slike sa anotacijama
        # cv2.imshow("YOLOv8 - Detekcija", frame)
        # print(frame.shape)


