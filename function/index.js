/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: LicenseRef-.amazon.com.-ASL-1.0
 * Licensed under the Amazon Software License (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *     http://aws.amazon.com/asl/
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
 
const AWS = require('aws-sdk')

AWS.config.region = process.env.AWS_REGION
const IOT_THING_NAME = process.env.IOT_THING_NAME
const IOT_ATS_ENDPOINT = process.env.IOT_ATS_ENDPOINT

const leds_on = '"leds": [{"red": 15,"green": 0,"blue": 0},{"red": 0,"green": 15,"blue": 0},{"red": 0,"green": 0,"blue": 15},{"red": 15,"green": 15,"blue": 0},{"red": 0,"green": 15,"blue": 15}]'
const leds_off = '"leds": [{"red": 0,"green": 0,"blue": 0},{"red": 0,"green": 0,"blue": 0},{"red": 0,"green": 0,"blue": 0},{"red": 0,"green": 0,"blue": 0},{"red": 0,"green": 0,"blue": 0}]'

exports.handler = async (request, context) => {
    if (request.directive.header.namespace === 'Alexa.Discovery' && request.directive.header.name === 'Discover') {
        log("DEBUG:", "Discover request",  JSON.stringify(request));
        handleDiscovery(request, context, "");
    } else if (request.directive.header.namespace === 'Alexa.PowerController') {
        if (request.directive.header.name === 'TurnOn' || request.directive.header.name === 'TurnOff') {
            log("DEBUG:", "TurnOn or TurnOff Request", JSON.stringify(request));
            await handlePowerControl(request, context);
        }
    }
}

const handleDiscovery = (request, context) => {
    var payload = {
        "endpoints":
        [
            {
                "endpointId": "demo_id",
                "manufacturerName": "Smart Device Company",
                "friendlyName": "Light Switch",
                "description": "Smart Device Switch",
                "displayCategories": ["SWITCH"],
                "cookie": {
                },
                "capabilities":
                [
                    {
                      "type": "AlexaInterface",
                      "interface": "Alexa",
                      "version": "3"
                    },
                    {
                        "interface": "Alexa.PowerController",
                        "version": "3",
                        "type": "AlexaInterface",
                        "properties": {
                            "supported": [{
                                "name": "powerState"
                            }],
                             "retrievable": true
                        }
                    }
                ]
            }
        ]
    };
    var header = request.directive.header;
    header.name = "Discover.Response";
    log("DEBUG", "Discovery Response: ", JSON.stringify({ header: header, payload: payload }));
    context.succeed({ event: { header: header, payload: payload } });
}

const log = (message, message1, message2) => {
    console.log(message + message1 + message2);
}

const toggleLight = (state) => {
    console.log(`toggling light to ${state}`)

    return new Promise((resolve, reject) => {
        const desired_state = state === 'ON' ? leds_on : leds_off
        console.log(`desired_state: ${desired_state}`)

        var params = {
          payload: `{"state":{"desired":{${desired_state}}}}`,
          thingName: IOT_THING_NAME
        }

        console.log(`updateThingShadow params: ${params}`)

        const iotdata = new AWS.IotData({endpoint: IOT_ATS_ENDPOINT})
        iotdata.updateThingShadow(params, (err, data) => {
          if (err){
            console.log(err, err.stack)
            reject(`Failed to update thing shadow: ${err.errorMessage}`)
          }else{
            console.log(`update thing shadow response: ${JSON.stringify(data)}`)
            resolve({"response": data})
          }
        })
    })
}

const handlePowerControl = async (request, context) => {
    // get device ID passed in during discovery
    var requestMethod = request.directive.header.name;
    var responseHeader = request.directive.header;
    responseHeader.namespace = "Alexa";
    responseHeader.name = "Response";
    responseHeader.messageId = responseHeader.messageId + "-R";
    // get user token pass in request
    var requestToken = request.directive.endpoint.scope.token;
    var powerResult;

    if (requestMethod === "TurnOn") {
        // Make the call to your device cloud for control
        // powerResult = stubControlFunctionToYourCloud(endpointId, token, request);
        powerResult = "ON";
    } else if (requestMethod === "TurnOff") {
        // Make the call to your device cloud for control and check for success
        // powerResult = stubControlFunctionToYourCloud(endpointId, token, request);
        powerResult = "OFF";
    }

    await toggleLight(powerResult)
        .then((result) => {
            console.log(`Toggle Light result: ${JSON.stringify(result)}`)
          //TODO - with previous examples, I was able to
          // emit responses with `speak` or `cardRenderer`
        }).catch((err) => {
            console.log(`Failed to toggle light: ${err}`)
          //TODO - with previous examples, I was able to
          // emit responses with `speak` or `cardRenderer`
        })

    var contextResult = {
        "properties": [{
            "namespace": "Alexa.PowerController",
            "name": "powerState",
            "value": powerResult,
            "timeOfSample": "2017-09-03T16:20:50.52Z", //retrieve from result.
            "uncertaintyInMilliseconds": 50
        }]
    };
    var response = {
        context: contextResult,
        event: {
            header: responseHeader,
            endpoint: {
                scope: {
                    type: "BearerToken",
                    token: requestToken
                },
                endpointId: "demo_id"
            },
            payload: {}
        }
    };
    log("DEBUG", "Alexa.PowerController ", JSON.stringify(response));
    context.succeed(response);
}
