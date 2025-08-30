# A fun DIY project: Flipdot panel

## The idea - The birth of the flipdot panel

In 2022, in Los Angeles, I saw the work of a NYC art studio called Breakfaststudio. And I was blown away. In the age of IMAX, 8k, 4k displays they chose to make art display with binary state dots panels. They are really doing nice things. Check their work [here](https://breakfaststudio.com) or on YouTube.

I thought it was a cool idea to replicate and a very cool DIY project. So I started to look around and gather information on the Internet.

## Technical parts of the project

### Panels

Not many companies are making flipdots panels nowadays. One of the leader is AlfaZeta in Poland.

They are providing panels in different size. I picked the 28x14 dots model (which is in fact two base 28x7 panels assembled together on a single board).

So one panel is a 28x14 dots matrix. And each pannel has two controllers per pannel.

Panels are driven with RS485 serial protocol. For small setup, one RS485 main controller interface can drive up to eight panels controllers. Which in my case means four panels per main controller interfaces.

### Display frame structure

The frame to send to the panel controller to display information is:

0x80: start of the frame

0x83: command (there are several commands but this one instruct to display on the panel and refresh it fully)

addres of the panel: each panel has and must have its own address (you setup the panel address with dip switches on it)

data: One byte is a stripe of dots (so 7 dots). Least Significant Bit (LSB) is the upper dot. Most Significant Bit (the 7th) is the lower dot. (MSB 8th is ignored and should be put to zero)

0x8F: end of the frame

### Panel display structure

I ended up with two RS485 lines, each with eight controllers to manage.

A visual representation of what it is.

```bash
[Ctrl1]Ctrl2]
[1]    [9]
[2]    [10]
[3]    [11]
[4]    [12]
[5]    [13]
[6]    [14]
[7]    [15]
[8]    [16]
```

And all of that is making a 56x56 dots matrix.

Then it's just a matter of matricial operations to display things on the panels by splitting what you want to display over the eight panels / sixteen controllers.

### Hand gesture recognition

Displaying things on the panel is fun but displaying things "live" or be able to interact with the panel is even funnier.

I am using a Jetson Nano. This is a small computer board that come with a small Nvidia GPU. The power of the GPU is sufficient to offload video detection for my use case.

I am using OpenCV to preprocess the images frames captured by the camera, then I use Google Mediapipe to handle hand detection. If "Open_Palm" is detected (which means to full hand open) for more than one second than hand detection mechanism is triggered. In my case it allows to move the image displayed on the panel.

Recognition Flow.

1. Frame Capture → Camera input (640x480)
2. Preprocessing → Flip, resize to 320x240, BGR→RGB conversion
3. MediaPipe Processing → Hand detection → landmark extraction → gesture classification
4. Confidence Check → Filter by detection/presence/tracking/gesture confidence
5. Temporal Smoothing → Consensus voting across frame window
6. State Change Detection → Compare with previous state
7. Message Broadcasting → ZMQ publish if state changed

## The hardware

### Computing

Jetson Nano with:
* a camera
* two USB to RS485 controllers
* WIFI interface

![Jetson](project_pics/jetson.jpg)

### Panels

Flidot panels are AlfaZeta panels, 28x14 dots size

## Some pictures of the construction

![Panels](project_pics/panels.jpg)

![Frame](project_pics/mounting.jpg)


## Some examples of what you can get
![Oasis_25](project_pics/oasis.jpg)

![LCD](project_pics/lcd.jpg)

![Hand_move](project_pics/hand_move.mov)