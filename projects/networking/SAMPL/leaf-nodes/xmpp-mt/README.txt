This program periodically sends an XMPP message to a JID
specified in the source.  If an XMPP message is sent back 
while the node is connected, it will print out the 
contents and sender information.

Before using this make sure to:
  1) Set your mobile node's correct MAC address
  2) Set your mobile node's password
  3) Set the destination JID and message

You can search in main.c for:
  // CHANGE THIS FIRST!
which will highlight where to make the changes.
