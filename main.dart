import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:get/get.dart';
import '/blecontroller.dart';

late BleController ble;
void main() => runApp(GetMaterialApp(home: Home()));

class Home extends StatelessWidget {
  @override
  Widget build(BuildContext bc) {
    ble = Get.put(BleController());
    return Scaffold(
        appBar: AppBar(title: Text('Control Remoto')),
        body: SingleChildScrollView(
            child: Column(children: [
          SizedBox(height: 20),
          ElevatedButton(
              onPressed: ble.connect,
              child: Obx(() => Text('${ble.status.value}'))),
          SizedBox(height: 20),
          GridView(
              shrinkWrap: true,
              gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
                  crossAxisCount: 3),
              primary: false,
              padding: const EdgeInsets.all(20),
              children: [
//1st row
                content(0x00, " ", Colors.white),
                content(0x66, "Derecho", Colors.green[200]),
                content(0x00, " ", Colors.white),

//2nd row
                content(0x6c, "Izquierda", Colors.teal[200]),
                content(0x73, "Parar", Colors.red[200]),
                content(0x72, "Derecha", Colors.teal[200]),

//3rd row
                content(0x00, " ", Colors.white),
                content(0x62, "Retroceder", Colors.orange[200]),
                content(0x00, " ", Colors.white),
              ])
        ])));
  }

  Widget content(data, text, color) {
    return InkWell(
        child: Container(
            child: Text(text), color: color, padding: EdgeInsets.all(8)),
        onTap: () => ble.send(data));
  }
}
