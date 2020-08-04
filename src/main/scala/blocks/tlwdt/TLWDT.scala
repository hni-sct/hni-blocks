// See LICENSE.SiFive for license details.

package hni.blocks.wdt

//import Chisel._
import hni.blocks.devices.wdarray._

import Chisel._
//import chisel3.util._
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.subsystem.BaseSubsystem
import freechips.rocketchip.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.devices.tilelink.CanHavePeripheryCLINT
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import chisel3.experimental._

case class WDTParams(regWidth: Int = 32, address: BigInt = 0x2000 , useAXI4: Boolean = false, Controllers : Int = 4, Outputs : Int = 4, Interrupts : Int = 3)  // regWidth in Bits
{
  def wdogOffset: Int = 0x0
  def wdogControllers: Int = Controllers
  def wdogOutputs : Int = Outputs
}

case object WDTKey extends Field[Option[WDTParams]](None)

class WDIO(n : Int) extends Bundle {
  val rst = (Vec(n,Bool(OUTPUT)))
  val clock = Clock(INPUT)
}

trait WDTBundle
{
  val params: WDTParams
  val c = params
  //val wdog_rst = Bool(OUTPUT)

  // Wdog MUX
  val wdog = new WDIO(c.wdogControllers)
}

trait WDTModule extends HasRegMap
{
  val params: WDTParams
  val io: WDTBundle
  val interrupts: Vec[Bool]
  val c = params
  //val state = RegInit(UInt(0, width = params.num))
  //val pending = RegInit(UInt(0xf, width = 4))

  //io.gpio := state
  //interrupts := pending.toBools

  // We use external Clock -> Connect externally to the Clock you want
  withClock(io.wdog.clock){
    val wdog = Module(new WatchdogArray(c.wdogControllers,c.wdogOutputs ,c.wdogOffset))
    //io.wdog_rst := wdog.io.outputs(0)
    for(i<- 0 until c.wdogControllers){
      io.wdog.rst(i) := wdog.io.outputs(i)
    }
    wdog.io.corerst := false.B  // has to be false or true???
    //for(i<-0 until Interrupts){
      interrupts := wdog.io.interrupts
    //}
    regmap(
      (WatchdogArray.arrRegMap(wdog,c.wdogOffset,4,c.wdogControllers, 40) :_*)
    ) 
  }
}

// Create a concrete TL2 version of the abstract Example slave
class TLWDT( params: WDTParams, beatBytes:Int)(implicit p: Parameters)
  extends TLRegisterRouter(
  params.address,
  "wdogarray", 
  Seq("hni,WatchdogArray"), 
  beatBytes = beatBytes,
  interrupts = params.Interrupts,
  concurrency = 1)(//interrupts = params.wdogControllers , 
  new TLRegBundle(params, _)    with WDTBundle)(
  new TLRegModule(params, _, _) with WDTModule) //with HasInterruptSources

class AXI4WDT(params: WDTParams, beatBytes: Int)(implicit p: Parameters)
  extends AXI4RegisterRouter(
    params.address,
    beatBytes=beatBytes,
    interrupts = params.Interrupts,
    concurrency = 1)(
      new AXI4RegBundle(params, _) with WDTBundle)(
      new AXI4RegModule(params, _, _) with WDTModule)

// java -jar rocket-chip/sbt-launch.jar ++2.12.4 "runMain freechips.rocketchip.system.Generator /home/julian/RISCV/freedom-e/builds/e300artydevkit sifive.freedom.everywhere.e300artydevkit E300ArtyDevKitFPGAChip sifive.freedom.everywhere.e300artydevkit E300ArtyDevKitConfig"
trait CanHavePeripheryWDT 
{ this: BaseSubsystem =>
  private val portName = "wdogarray"

  // Only build if we are using the TL (nonAXI4) version
  val crossing = LazyModule(new TLAsyncCrossingSink(AsyncQueueParams.singleton()))
  val wdt = p(WDTKey) match {
    case Some(params) => {
      if (params.useAXI4) {
        val wdt = LazyModule(new AXI4WDT(params, pbus.beatBytes)(p))
        pbus.toSlave(Some(portName)) {
          wdt.node :=
          AXI4Buffer () :=
          TLToAXI4 () :=
          crossing.node :=
          TLAsyncCrossingSource() :=
          // toVariableWidthSlave doesn't use holdFirstDeny, which TLToAXI4() needsx
          TLFragmenter(pbus.beatBytes, pbus.blockBytes, holdFirstDeny = true)
          
        }
        //val crossing = LazyModule(new TLAsyncCrossingSink(AsyncQueueParams.singleton()))
        ibus.fromSync := IntSyncCrossingSink() := IntSyncCrossingSource(alreadyRegistered = true)  := wdt.intnode // Ibus -> Connecting to Interruptsystem
        Some(wdt)
      } else {
        val wdt = LazyModule(new TLWDT(params, pbus.beatBytes)(p))
        val node = wdt.node := crossing.node 
        pbus.toVariableWidthSlave(Some(portName)) { node := TLAsyncCrossingSource() }  // Pbus -> Connecting to PeripherialBus // sbus -> Systembus
        ibus.fromSync := IntSyncCrossingSink() := IntSyncCrossingSource(alreadyRegistered = true)  := wdt.intnode // Ibus -> Connecting to Interruptsystem
        Some(wdt)
      }
      //val outputs = params.wdogOutputs
    }
    case None => None
  }
}

trait CanHavePeripheryWDTModuleImp extends LazyModuleImp {
  val outer: CanHavePeripheryWDT
  //val params: WDTParams
  
  val wdt_out = outer.wdt match {
    case Some(wdt) => {
      val c = wdt.module.params
      val wdog = IO(new WDIO(  c.wdogOutputs ))
      wdog.rst := wdt.module.io.wdog.rst
      wdt.module.io.wdog.clock := wdog.clock
      outer.crossing.module.clock := wdog.clock
      Some(wdog)
    }
    case None => None
  }
}

class WithWDT(address: BigInt = 0x2000, useAXI4: Boolean, Controllers : Int = 4, Outputs : Int = 4, Interrupts : Int = 3) extends Config((site, here, up) => {
  case WDTKey => Some(WDTParams(address = address, useAXI4 = useAXI4, Controllers = Controllers, Outputs = Outputs, Interrupts = Interrupts))
})
